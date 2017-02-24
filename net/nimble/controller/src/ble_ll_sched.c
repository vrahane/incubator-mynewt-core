/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "os/os.h"
#include "os/os_cputime.h"
#include "ble/xcvr.h"
#include "controller/ble_phy.h"
#include "controller/ble_ll.h"
#include "controller/ble_ll_sched.h"
#include "controller/ble_ll_adv.h"
#include "controller/ble_ll_scan.h"
#include "ble_ll_conn_priv.h"

/* XXX: this is temporary. Not sure what I want to do here */
struct hal_timer g_ble_ll_sched_timer;

#define BLE_LL_SCHED_ADV_WORST_CASE_USECS       \
    (BLE_LL_SCHED_MAX_ADV_PDU_USECS + BLE_LL_IFS + BLE_LL_SCHED_ADV_MAX_USECS \
     + XCVR_TX_SCHED_DELAY_USECS)


#if (BLE_LL_SCHED_DEBUG == 1)
int32_t g_ble_ll_sched_max_late;
#endif

/* XXX: TODO:
 *  1) Add some accounting to the schedule code to see how late we are
 *  (min/max?)
 *
 *  2) Need to determine how we really want to handle the case when we execute
 *  a schedule item but there is a current event. We could:
 *      -> Reschedule the schedule item and let current event finish
 *      -> Kill the current event and run the scheduled item.
 *      -> Disable schedule timer while in an event; could cause us to be late.
 *      -> Wait for current event to finish hoping it does before schedule item.
 */

/* Queue for timers */
TAILQ_HEAD(ll_sched_qhead, ble_ll_sched_item) g_ble_ll_sched_q;

/**
 * Checks if two events in the schedule will overlap in time. NOTE: consecutive
 * schedule items can end and start at the same time.
 *
 * @param s1
 * @param s2
 *
 * @return int 0: dont overlap 1:overlap
 */
static int
ble_ll_sched_is_overlap(struct ble_ll_sched_item *s1,
                        struct ble_ll_sched_item *s2)
{
    int rc;

    rc = 1;
    if ((int32_t)(s1->start_time - s2->start_time) < 0) {
        /* Make sure this event does not overlap current event */
        if ((int32_t)(s1->end_time - s2->start_time) <= 0) {
            rc = 0;
        }
    } else {
        /* Check for overlap */
        if ((int32_t)(s1->start_time - s2->end_time) >= 0) {
            rc = 0;
        }
    }

    return rc;
}

/*
 * Determines if the schedule item overlaps the currently running schedule
 * item. We only care about connection schedule items
 */
int
ble_ll_sched_overlaps_current(struct ble_ll_sched_item *sch)
{
    int rc;
    uint32_t ce_end_time;

    rc = 0;
    if (ble_ll_state_get() == BLE_LL_STATE_CONNECTION) {
        ce_end_time = ble_ll_conn_get_ce_end_time();
        if ((int32_t)(ce_end_time - sch->start_time) > 0) {
            rc = 1;
        }
    }
    return rc;
}

static int
ble_ll_sched_conn_overlap(struct ble_ll_sched_item *entry)
{
    int rc;
    struct ble_ll_conn_sm *connsm;

    /* Should only be advertising or a connection here */
    if (entry->sched_type == BLE_LL_SCHED_TYPE_CONN) {
        connsm = (struct ble_ll_conn_sm *)entry->cb_arg;
        entry->enqueued = 0;
        TAILQ_REMOVE(&g_ble_ll_sched_q, entry, link);
        ble_ll_event_send(&connsm->conn_ev_end);
        rc = 0;
    } else {
        rc = -1;
    }

    return rc;
}

struct ble_ll_sched_item *
ble_ll_sched_insert_if_empty(struct ble_ll_sched_item *sch)
{
    struct ble_ll_sched_item *entry;

    entry = TAILQ_FIRST(&g_ble_ll_sched_q);
    if (!entry) {
        TAILQ_INSERT_HEAD(&g_ble_ll_sched_q, sch, link);
        sch->enqueued = 1;
    }
    return entry;
}

int
ble_ll_sched_conn_reschedule(struct ble_ll_conn_sm *connsm)
{
    int rc;
    os_sr_t sr;
    uint32_t usecs;
    struct ble_ll_sched_item *sch;
    struct ble_ll_sched_item *start_overlap;
    struct ble_ll_sched_item *end_overlap;
    struct ble_ll_sched_item *entry;
    struct ble_ll_conn_sm *tmp;

    /* Get schedule element from connection */
    sch = &connsm->conn_sch;

    /* Set schedule start and end times */
    if (connsm->conn_role == BLE_LL_CONN_ROLE_SLAVE) {
        usecs = XCVR_RX_SCHED_DELAY_USECS;
        usecs += connsm->slave_cur_window_widening;
    } else {
        usecs = XCVR_TX_SCHED_DELAY_USECS;
    }
    sch->start_time = connsm->anchor_point - os_cputime_usecs_to_ticks(usecs);
    sch->end_time = connsm->ce_end_time;

    /* Better be past current time or we just leave */
    if ((int32_t)(sch->start_time - os_cputime_get32()) < 0) {
        return -1;
    }

    /* We have to find a place for this schedule */
    OS_ENTER_CRITICAL(sr);

    if (ble_ll_sched_overlaps_current(sch)) {
        OS_EXIT_CRITICAL(sr);
        return -1;
    }

    /* Stop timer since we will add an element */
    os_cputime_timer_stop(&g_ble_ll_sched_timer);

    start_overlap = NULL;
    end_overlap = NULL;
    rc = 0;
    TAILQ_FOREACH(entry, &g_ble_ll_sched_q, link) {
        if (ble_ll_sched_is_overlap(sch, entry)) {
            /* Only insert if this element is older than all that we overlap */
            if (!ble_ll_conn_is_lru((struct ble_ll_conn_sm *)sch->cb_arg,
                                    (struct ble_ll_conn_sm *)entry->cb_arg)) {
                start_overlap = NULL;
                rc = -1;
                break;
            }
            if (start_overlap == NULL) {
                start_overlap = entry;
                end_overlap = entry;
            } else {
                end_overlap = entry;
            }
        } else {
            if ((int32_t)(sch->end_time - entry->start_time) <= 0) {
                rc = 0;
                TAILQ_INSERT_BEFORE(entry, sch, link);
                break;
            }
        }
    }

    if (!rc) {
        if (!entry) {
            TAILQ_INSERT_TAIL(&g_ble_ll_sched_q, sch, link);
        }
        sch->enqueued = 1;
    }

    /* Remove first to last scheduled elements */
    entry = start_overlap;
    while (entry) {
        start_overlap = TAILQ_NEXT(entry,link);
        if (entry->sched_type == BLE_LL_SCHED_TYPE_CONN) {
            tmp = (struct ble_ll_conn_sm *)entry->cb_arg;
            ble_ll_event_send(&tmp->conn_ev_end);
        } else {
            assert(entry->sched_type == BLE_LL_SCHED_TYPE_ADV);
            ble_ll_adv_event_rmvd_from_sched((struct ble_ll_adv_sm *)entry->cb_arg);
        }

        TAILQ_REMOVE(&g_ble_ll_sched_q, entry, link);
        entry->enqueued = 0;

        if (entry == end_overlap) {
            break;
        }
        entry = start_overlap;
    }

    /* Get first on list */
    sch = TAILQ_FIRST(&g_ble_ll_sched_q);

    OS_EXIT_CRITICAL(sr);

    /* Restart timer */
    os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);

    return rc;
}

int
ble_ll_sched_master_new(struct ble_ll_conn_sm *connsm, uint32_t adv_rxend,
                        uint8_t req_slots)
{
    int rc;
    os_sr_t sr;
    uint32_t tps;
    uint32_t initial_start;
    uint32_t earliest_start;
    uint32_t earliest_end;
    uint32_t dur;
    uint32_t itvl_t;
    uint32_t ce_end_time;
    struct ble_ll_sched_item *entry;
    struct ble_ll_sched_item *sch;

    /* Better have a connsm */
    assert(connsm != NULL);

    /* Get schedule element from connection */
    rc = -1;
    sch = &connsm->conn_sch;

    /*
     * The earliest start time is 1.25 msecs from the end of the connect
     * request transmission. Note that adv_rxend is the end of the received
     * advertisement, so we need to add an IFS plus the time it takes to send
     * the connection request
     */
    dur = os_cputime_usecs_to_ticks(req_slots * BLE_LL_SCHED_USECS_PER_SLOT);
    earliest_start = adv_rxend +
        os_cputime_usecs_to_ticks(BLE_LL_IFS + BLE_LL_CONN_REQ_DURATION +
                                  BLE_LL_CONN_INITIAL_OFFSET);
    earliest_end = earliest_start + dur;

    itvl_t = os_cputime_usecs_to_ticks(connsm->conn_itvl * BLE_LL_CONN_ITVL_USECS);

    /* We have to find a place for this schedule */
    OS_ENTER_CRITICAL(sr);

    /* The schedule item must occur after current running item (if any) */
    sch->start_time = earliest_start;

    /*
     * If we are currently in a connection, we add one slot time to the
     * earliest start so we can end the connection reasonably.
     */
    if (ble_ll_state_get() == BLE_LL_STATE_CONNECTION) {
        tps = os_cputime_usecs_to_ticks(BLE_LL_SCHED_USECS_PER_SLOT);
        ce_end_time = ble_ll_conn_get_ce_end_time();
        while ((int32_t)(ce_end_time - os_cputime_get32()) < 0) {
            ce_end_time += tps;
        }

        /* Start at next slot boundary past earliest */
        while ((int32_t)(ce_end_time - earliest_start) < 0) {
            ce_end_time += tps;
        }
        earliest_start = ce_end_time;
        earliest_end = earliest_start + dur;
    }
    initial_start = earliest_start;

    if (!ble_ll_sched_insert_if_empty(sch)) {
        /* Nothing in schedule. Schedule as soon as possible */
        rc = 0;
        connsm->tx_win_off = 0;
    } else {
        os_cputime_timer_stop(&g_ble_ll_sched_timer);
        TAILQ_FOREACH(entry, &g_ble_ll_sched_q, link) {
            /* Set these because overlap function needs them to be set */
            sch->start_time = earliest_start;
            sch->end_time = earliest_end;

            /* We can insert if before entry in list */
            if ((int32_t)(sch->end_time - entry->start_time) <= 0) {
                if ((earliest_start - initial_start) <= itvl_t) {
                    rc = 0;
                    TAILQ_INSERT_BEFORE(entry, sch, link);
                }
                break;
            }

            /* Check for overlapping events */
            if (ble_ll_sched_is_overlap(sch, entry)) {
                /* Earliest start is end of this event since we overlap */
                earliest_start = entry->end_time;
                earliest_end = earliest_start + dur;
            }
        }

        if (!entry) {
            if ((earliest_start - initial_start) <= itvl_t) {
                rc = 0;
                TAILQ_INSERT_TAIL(&g_ble_ll_sched_q, sch, link);
            }
        }

        if (!rc) {
            /* calculate number of connection intervals before start */
            sch->enqueued = 1;
            connsm->tx_win_off = (earliest_start - initial_start) /
                os_cputime_usecs_to_ticks(BLE_LL_CONN_ITVL_USECS);
        }
    }

    if (!rc) {
        sch->start_time = earliest_start;
        sch->end_time = earliest_end;
        connsm->anchor_point = earliest_start +
            os_cputime_usecs_to_ticks(XCVR_TX_SCHED_DELAY_USECS);
        connsm->ce_end_time = earliest_end;
    }

    /* Get head of list to restart timer */
    sch = TAILQ_FIRST(&g_ble_ll_sched_q);

    OS_EXIT_CRITICAL(sr);

    os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);

    return rc;
}

int
ble_ll_sched_slave_new(struct ble_ll_conn_sm *connsm)
{
    int rc;
    os_sr_t sr;
    struct ble_ll_sched_item *entry;
    struct ble_ll_sched_item *next_sch;
    struct ble_ll_sched_item *sch;

    /* Get schedule element from connection */
    rc = -1;
    sch = &connsm->conn_sch;

    /* Set schedule start and end times */
    sch->start_time = connsm->anchor_point -
        os_cputime_usecs_to_ticks(XCVR_RX_SCHED_DELAY_USECS +
                                  connsm->slave_cur_window_widening);
    sch->end_time = connsm->ce_end_time;

    /* We have to find a place for this schedule */
    OS_ENTER_CRITICAL(sr);

    /* The schedule item must occur after current running item (if any) */
    if (ble_ll_sched_overlaps_current(sch)) {
        OS_EXIT_CRITICAL(sr);
        return rc;
    }

    entry = ble_ll_sched_insert_if_empty(sch);
    if (!entry) {
        /* Nothing in schedule. Schedule as soon as possible */
        rc = 0;
    } else {
        os_cputime_timer_stop(&g_ble_ll_sched_timer);
        while (1) {
            next_sch = entry->link.tqe_next;
            /* Insert if event ends before next starts */
            if ((int32_t)(sch->end_time - entry->start_time) <= 0) {
                rc = 0;
                TAILQ_INSERT_BEFORE(entry, sch, link);
                break;
            }

            if (ble_ll_sched_is_overlap(sch, entry)) {
                /* If we overlap with a connection, we re-schedule */
                if (ble_ll_sched_conn_overlap(entry)) {
                    break;
                }
            }

            /* Move to next entry */
            entry = next_sch;

            /* Insert at tail if none left to check */
            if (!entry) {
                rc = 0;
                TAILQ_INSERT_TAIL(&g_ble_ll_sched_q, sch, link);
                break;
            }
        }

        if (!rc) {
            sch->enqueued = 1;
        }
        sch = TAILQ_FIRST(&g_ble_ll_sched_q);
    }

    OS_EXIT_CRITICAL(sr);

    os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);

    return rc;
}

int
ble_ll_sched_adv_new(struct ble_ll_sched_item *sch)
{
    int rc;
    os_sr_t sr;
    uint8_t ll_state;
    int32_t ticks;
    uint32_t ce_end_time;
    uint32_t adv_start;
    uint32_t duration;
    struct ble_ll_sched_item *entry;
    struct ble_ll_sched_item *orig;

    /* Get length of schedule item */
    duration = sch->end_time - sch->start_time;
    orig = sch;

    OS_ENTER_CRITICAL(sr);

    /*
     * If we are currently in a connection, we add one slot time to the
     * earliest start so we can end the connection reasonably.
     */
    ll_state = ble_ll_state_get();
    if (ll_state == BLE_LL_STATE_CONNECTION) {
        ticks = (int32_t)os_cputime_usecs_to_ticks(BLE_LL_SCHED_MAX_TXRX_SLOT);
        ce_end_time = ble_ll_conn_get_ce_end_time();
        if ((int32_t)(ce_end_time - sch->start_time) < ticks) {
            ce_end_time += ticks;
        }
        sch->start_time = ce_end_time;
        sch->end_time = ce_end_time + duration;
    }
#if MYNEWT_VAL(BLE_MULTI_ADV_SUPPORT)
    else if ((ll_state == BLE_LL_STATE_ADV) && (BLE_LL_ADV_INSTANCES > 1)) {
        /*
         * Since we currently dont know how long this item might be scheduled
         * for we add what we think the worst-case time for the advertising
         * scheduled item to be over. We add in a IFS for good measure.
         */
        sch->start_time += BLE_LL_SCHED_MAX_ADV_PDU_USECS + BLE_LL_IFS +
            BLE_LL_SCHED_ADV_MAX_USECS + XCVR_TX_SCHED_DELAY_USECS;
        sch->end_time = sch->start_time + duration;
    }
#endif

    entry = ble_ll_sched_insert_if_empty(sch);
    if (!entry) {
        rc = 0;
        adv_start = sch->start_time;
    } else {
        os_cputime_timer_stop(&g_ble_ll_sched_timer);
        TAILQ_FOREACH(entry, &g_ble_ll_sched_q, link) {
            /* We can insert if before entry in list */
            if ((int32_t)(sch->end_time - entry->start_time) <= 0) {
                rc = 0;
                TAILQ_INSERT_BEFORE(entry, sch, link);
                break;
            }

            /* Check for overlapping events */
            if (ble_ll_sched_is_overlap(sch, entry)) {
                /* Earliest start is end of this event since we overlap */
                sch->start_time = entry->end_time;
                sch->end_time = sch->start_time + duration;
            }
        }

        if (!entry) {
            rc = 0;
            TAILQ_INSERT_TAIL(&g_ble_ll_sched_q, sch, link);
        }
        adv_start = sch->start_time;

        if (!rc) {
            sch->enqueued = 1;
        }

        /* Restart with head of list */
        sch = TAILQ_FIRST(&g_ble_ll_sched_q);
    }

    ble_ll_adv_scheduled((struct ble_ll_adv_sm *)orig->cb_arg, adv_start);

    OS_EXIT_CRITICAL(sr);

    /* XXX: some things to test. I am not sure that if we are passed the
       output compare that we actually get the interrupt. */
    /* XXX: I am not sure that if we receive a packet while scanning
     * that we actually go back to scanning. I need to make sure
       we re-enable the receive. Put an event in the log! */

    os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);

    return rc;
}

int
ble_ll_sched_adv_reschedule(struct ble_ll_sched_item *sch, uint32_t *start,
                            uint32_t max_delay_ticks)
{
    int rc;
    os_sr_t sr;
    uint32_t orig_start;
    uint32_t duration;
    uint32_t rand_ticks;
    struct ble_ll_sched_item *entry;
    struct ble_ll_sched_item *next_sch;
    struct ble_ll_sched_item *before;
    struct ble_ll_sched_item *start_overlap;
    struct ble_ll_sched_item *end_overlap;

    /* Get length of schedule item */
    duration = sch->end_time - sch->start_time;

    /* Add maximum randomization delay to end */
    rand_ticks = max_delay_ticks;
    sch->end_time += max_delay_ticks;

    start_overlap = NULL;
    before = NULL;
    rc = 0;
    OS_ENTER_CRITICAL(sr);

    entry = ble_ll_sched_insert_if_empty(sch);
    if (entry) {
        os_cputime_timer_stop(&g_ble_ll_sched_timer);
        while (1) {
            next_sch = entry->link.tqe_next;
            if (ble_ll_sched_is_overlap(sch, entry)) {
                if (start_overlap == NULL) {
                    start_overlap = entry;
                    end_overlap = entry;
                } else {
                    end_overlap = entry;
                }
            } else {
                if ((int32_t)(sch->end_time - entry->start_time) <= 0) {
                    before = entry;
                    break;
                }
            }

            entry = next_sch;
            if (entry == NULL) {
                break;
            }
        }

        /*
         * If there is no overlap, we either insert before the 'before' entry
         * or we insert at the end if there is no before entry.
         */
        if (start_overlap == NULL) {
            if (before) {
                TAILQ_INSERT_BEFORE(before, sch, link);
            } else {
                TAILQ_INSERT_TAIL(&g_ble_ll_sched_q, sch, link);
            }
        } else {
            /*
             * This item will overlap with others. See if we can fit it in
             * with original duration.
             */
            before = NULL;
            orig_start = sch->start_time;
            entry = start_overlap;
            sch->end_time = sch->start_time + duration;
            while (1) {
                next_sch = entry->link.tqe_next;
                if ((int32_t)(sch->end_time - entry->start_time) <= 0) {
                    rand_ticks = entry->start_time - sch->end_time;
                    before = entry;
                    TAILQ_INSERT_BEFORE(before, sch, link);
                    break;
                } else {
                    sch->start_time = entry->end_time;
                    sch->end_time = sch->start_time + duration;
                }

                if (entry == end_overlap) {
                    rand_ticks = (orig_start + max_delay_ticks) - sch->start_time;
                    if (rand_ticks > max_delay_ticks) {
                        /* No place for advertisement. */
                        rc = -1;
                    } else {
                        if (next_sch == NULL) {
                            TAILQ_INSERT_TAIL(&g_ble_ll_sched_q, sch, link);
                        } else {
                            TAILQ_INSERT_BEFORE(next_sch, sch, link);
                        }
                    }
                    break;
                }
                entry = next_sch;
                assert(entry != NULL);
            }
        }
    }

    if (!rc) {
        sch->enqueued = 1;
        if (rand_ticks) {
            sch->start_time += rand() % rand_ticks;
        }
        sch->end_time = sch->start_time + duration;
        *start = sch->start_time;
    }

    OS_EXIT_CRITICAL(sr);

    sch = TAILQ_FIRST(&g_ble_ll_sched_q);
    os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);

    return rc;
}

int
ble_ll_sched_adv_resched_pdu(struct ble_ll_sched_item *sch)
{
    uint8_t lls;
    os_sr_t sr;
    struct ble_ll_sched_item *entry;

    OS_ENTER_CRITICAL(sr);

    lls = ble_ll_state_get();
    if ((lls == BLE_LL_STATE_ADV) || (lls == BLE_LL_STATE_CONNECTION)) {
        goto adv_resched_pdu_fail;
    }

    entry = ble_ll_sched_insert_if_empty(sch);
    if (entry) {
        /* If we overlap with the first item, simply re-schedule */
        if (ble_ll_sched_is_overlap(sch, entry)) {
            goto adv_resched_pdu_fail;
        }
        os_cputime_timer_stop(&g_ble_ll_sched_timer);
        TAILQ_INSERT_BEFORE(entry, sch, link);
    }

    OS_EXIT_CRITICAL(sr);
    os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);
    return 0;

adv_resched_pdu_fail:
    OS_EXIT_CRITICAL(sr);
    return -1;
}

/**
 * Remove a schedule element
 *
 * @param sched_type
 *
 * @return int
 */
void
ble_ll_sched_rmv_elem(struct ble_ll_sched_item *sch)
{
    os_sr_t sr;
    struct ble_ll_sched_item *first;

    if (!sch) {
        return;
    }

    OS_ENTER_CRITICAL(sr);
    if (sch->enqueued) {
        first = TAILQ_FIRST(&g_ble_ll_sched_q);
        if (first == sch) {
            os_cputime_timer_stop(&g_ble_ll_sched_timer);
        }

        TAILQ_REMOVE(&g_ble_ll_sched_q, sch, link);
        sch->enqueued = 0;

        if (first == sch) {
            first = TAILQ_FIRST(&g_ble_ll_sched_q);
            if (first) {
                os_cputime_timer_start(&g_ble_ll_sched_timer, first->start_time);
            }
        }
    }
    OS_EXIT_CRITICAL(sr);
}

/**
 * Executes a schedule item by calling the schedule callback function.
 *
 * Context: Interrupt
 *
 * @param sch Pointer to schedule item
 *
 * @return int 0: schedule item is not over; otherwise schedule item is done.
 */
static int
ble_ll_sched_execute_item(struct ble_ll_sched_item *sch)
{
    int rc;
    uint8_t lls;

    /*
     * This is either an advertising event or connection event start. If
     * we are scanning or initiating just stop it.
     */
    lls = ble_ll_state_get();
    if (lls != BLE_LL_STATE_STANDBY) {
        /* We have to disable the PHY no matter what */
        ble_phy_disable();
        ble_ll_wfr_disable();
        if ((lls == BLE_LL_STATE_SCANNING) ||
            (lls == BLE_LL_STATE_INITIATING)) {
            ble_ll_state_set(BLE_LL_STATE_STANDBY);
        } else if (lls == BLE_LL_STATE_ADV) {
            STATS_INC(ble_ll_stats, sched_state_adv_errs);
            ble_ll_adv_halt((struct ble_ll_adv_sm *)sch->cb_arg);
        } else {
            STATS_INC(ble_ll_stats, sched_state_conn_errs);
            ble_ll_conn_event_halt();
        }
    }

    assert(sch->sched_cb);
    rc = sch->sched_cb(sch);
    return rc;
}

/**
 * Run the BLE scheduler. Iterate through all items on the schedule queue.
 *
 * Context: interrupt (scheduler)
 *
 * @return int
 */
void
ble_ll_sched_run(void *arg)
{
    int32_t dt;
    struct ble_ll_sched_item *sch;

    /* Look through schedule queue */
    while ((sch = TAILQ_FIRST(&g_ble_ll_sched_q)) != NULL) {
        /* Make sure we have passed the start time of the first event */
        dt = (int32_t)(os_cputime_get32() - sch->start_time);
        if (dt >= 0) {
#if (BLE_LL_SCHED_DEBUG == 1)
            if (dt > g_ble_ll_sched_max_late) {
                g_ble_ll_sched_max_late = dt;
            }
#endif
            /* Remove schedule item and execute the callback */
            TAILQ_REMOVE(&g_ble_ll_sched_q, sch, link);
            sch->enqueued = 0;
            ble_ll_sched_execute_item(sch);
        } else {
            os_cputime_timer_start(&g_ble_ll_sched_timer, sch->start_time);
            break;
        }
    }
}

/**
 * Called to determine when the next scheduled event will occur.
 *
 * If there are not scheduled events this function returns 0; otherwise it
 * returns 1 and *next_event_time is set to the start time of the next event.
 *
 * @param next_event_time
 *
 * @return int 0: No events are scheduled 1: there is an upcoming event
 */
int
ble_ll_sched_next_time(uint32_t *next_event_time)
{
    int rc;
    os_sr_t sr;
    struct ble_ll_sched_item *first;

    rc = 0;
    OS_ENTER_CRITICAL(sr);
    first = TAILQ_FIRST(&g_ble_ll_sched_q);
    if (first) {
        *next_event_time = first->start_time;
        rc = 1;
    }
    OS_EXIT_CRITICAL(sr);

    return rc;
}

/**
 * Stop the scheduler
 *
 * Context: Link Layer task
 */
void
ble_ll_sched_stop(void)
{
    os_cputime_timer_stop(&g_ble_ll_sched_timer);
}

/**
 * Initialize the scheduler. Should only be called once and should be called
 * before any of the scheduler API are called.
 *
 * @return int
 */
int
ble_ll_sched_init(void)
{
    /* Initialize cputimer for the scheduler */
    os_cputime_timer_init(&g_ble_ll_sched_timer, ble_ll_sched_run, NULL);
    return 0;
}
