import Vector::*;
import FIFO::*;
import FIFOF::*;
import ClientServer::*;
import SpecialFIFOs::*;
import BRAM::*;
import GetPut::*;
import DefaultValue::*;
import Clocks::*;
import PIEOQueue::*;

import ShaleUtil::*;
import SchedulerTypes::*;
import RingBufferTypes::*;
import RingBuffer::*;
import CellGenerator::*;
import Mac::*;

`include "ConnectalProjectConfig.bsv"

interface Scheduler;
    // Responses to stats request.
    interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
        time_slots_res;
	interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
        received_host_pkt_res;
	interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
        received_wrong_dst_pkt_res;
	interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(72)))
        latency_res;
    interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
        max_fwd_buffer_length_res;
    // interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
    //     sent_host_pkt_res;
	// interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
    //     sent_fwd_pkt_res;
	// interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
    //     received_fwd_pkt_res;
	// interface Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64)))
    //     received_corrupted_pkt_res;

    // Stats
    method Action timeSlotsCount();
	method Action receivedHostPktCount();
	method Action receivedWrongDstPktCount();
    method Action latency();
    method Action max_fwd_buffer_length();
    // method Action sentHostPktCount();
	// method Action sentFwdPktCount();
    // method Action receivedFwdPktCount();
	// method Action receivedCorruptedPktCount();

    method Action start(ServerIndex first_host_index, Bit#(8) t);
    method Action stop();
endinterface

module mkScheduler#(Mac mac, Vector#(NUM_OF_ALTERA_PORTS, CellGenerator) cell_generator,
                Clock pcieClock, Reset pcieReset) (Scheduler);

    Bool verbose = True;

    Clock defaultClock <- exposeCurrentClock();
    Reset defaultReset <- exposeCurrentReset();

    Reg#(Bit#(8)) timeslot_len <- mkReg(0);

    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(1))) start_flag <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(1))) running <- replicateM(mkReg(0));

    // Clock for stats.
    Reg#(Bit#(72)) current_time <- mkReg(0);
    rule clk (start_flag[0] == 1);
        current_time <= current_time + 1;
    endrule

/*------------------------------------------------------------------------------*/

                                /* Init Path */
            /* Takes 1 cycles to set coordinates + schedulte table. */

/*------------------------------------------------------------------------------*/

    // Node ID for each port.
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex)) host_index
        <- replicateM(mkReg(maxBound));

    // The schedule is a 3D matrix, indexed by host_idx, phase, slot_within_phase.
    // The table need only contain schedules for the nodes on this device.
	Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, Reg#(ServerIndex))))
        schedule_table <- replicateM(replicateM(replicateM(mkReg(0))));

    // For Tx. Destination node of the next local flow we will send out, when there's an opportunity.
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex)) 
        next_local_flow_idx <- replicateM(mkReg(0));

    `ifdef LIMIT_ACTIVE_BUCKETS
        // Map each overall token bucket idx to stored bucket idx. Unallocated
        // buckets have default value of max bkt + 1. Also map the inverse.
        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_TOKEN_BUCKETS, Reg#(Bit#(BUCKET_IDX_BITS))))
            token_bkt_map <- replicateM(replicateM(mkReg(fromInteger(valueof(NUM_ACTIVE_BUCKETS)))));
        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_ACTIVE_BUCKETS, Reg#(Bit#(BUCKET_IDX_BITS))))
            bkt_token_map <- replicateM(replicateM(mkReg(fromInteger(valueof(NUM_TOKEN_BUCKETS)))));
    `endif

    // Once on receiving start signal (and therefor host_index), set coordinates and load schedule into table for each port.
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(1))) is_schedule_set <- replicateM(mkReg(0));
    for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        rule populate_schedule_table (start_flag[i] == 1 && is_schedule_set[i] == 0);
            for (Integer phase = 0; phase < valueof(NUM_OF_PHASES); phase = phase + 1)  // Phase
            begin
                for (Integer j = 0; j < valueof(PHASE_SIZE); j = j + 1)                 // Slot within phase
                    schedule_table[i][phase][j] <= offset_node_in_phase(host_index[i], phase, j + 1);
            end

            // Init round robin for local flows to send.
            next_local_flow_idx[i] <= (host_index[i] + 1) % fromInteger(valueof(NUM_OF_SERVERS));

            // Init token bitmap
            `ifdef LIMIT_ACTIVE_BUCKETS
                token_bkt_map[i][valueof(FINAL_DST_BUCKET_IDX)] <= fromInteger(valueof(FINAL_DST_BUCKET_IDX));
                bkt_token_map[i][valueof(FINAL_DST_BUCKET_IDX)] <= fromInteger(valueof(FINAL_DST_BUCKET_IDX));
            `endif

            is_schedule_set[i] <= 1;
            
            // State is initialized, so now this port may run rx / tx / reconfig.
            if (verbose)
                $display("[SCHED %d] Init finished at abs time %d",host_index[i], current_time);
            running[i] <= 1;
        endrule
    end

    /* ---------------- Stats ----------------- */
	Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) time_slots_fifo
	        <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
	Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) received_host_pkt_fifo
	        <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
	
	Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) received_wrong_dst_pkt_fifo
	        <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
	Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(72))) latency_fifo
	        <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
    Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) buffer_length_fifo
	        <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
    // Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) sent_host_pkt_fifo
	//         <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
	// Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) sent_fwd_pkt_fifo
	//         <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
    // Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) received_fwd_pkt_fifo
	//         <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));
	// Vector#(NUM_OF_ALTERA_PORTS, SyncFIFOIfc#(Bit#(64))) received_corrupted_pkt_fifo
	//         <- replicateM(mkSyncFIFO(1, defaultClock, defaultReset, pcieClock));

	Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64))) num_of_time_slots_used_reg
        <- replicateM(mkReg(0));
	Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64)))
        num_of_host_pkt_received_reg <- replicateM(mkReg(0));
	Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64)))
        num_of_wrong_dst_pkt_received_reg <- replicateM(mkReg(0));
	Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(72)))
        latency_reg <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(8)))
        max_fwd_buffer_length_reg <- replicateM(mkReg(0));
    // Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64))) host_pkt_transmitted_reg
    //     <- replicateM(mkReg(0));
	// Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64))) non_host_pkt_transmitted_reg
    //     <- replicateM(mkReg(0));
    // Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64)))
    //     num_of_fwd_pkt_received_reg <- replicateM(mkReg(0));
	// Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(64)))
    //     num_of_corrupted_pkt_received_reg <- replicateM(mkReg(0));

/*------------------------------------------------------------------------------*/

                                /* Reconfigure Path */
            /* 1 cycle to reconfigure, executes every timeslot_len cycles */

/*------------------------------------------------------------------------------*/

    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(8)))
        clock_within_timeslot <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Coordinate))
        current_timeslot <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Phase))
        current_phase <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(7)))
        current_epoch <- replicateM(mkReg(0));

    // Tx neighbor.
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex))
        current_neighbor <- replicateM(mkReg(maxBound));

    // Signal FIFO to trigger choice of cell to send.
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, FIFO#(void))))
        choose_buffer_to_send_from_fifo <- replicateM(replicateM(replicateM(mkBypassFIFO)));
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, FIFO#(PIEOElement))))
        choose_buffer_to_send_from_2_fifo <- replicateM(replicateM(replicateM(mkBypassFIFO)));

    // Signal FIFO to enqueue fwd cells recvd.
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, FIFO#(PIEOElement))))
        add_pending_fwd_flow <- replicateM(replicateM(replicateM(mkFIFO)));

    for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        
        // Here, clock_within_timeslot counts cycles up to the timeslot length, repeatedly. At the start of
        // each time slot, it updates the neighbor acc to the schedule, and enqueues a signal to choose 
        // the tx cell for the time slot. At the end of a timeslot, it increments the time slot and optionally 
        // the phase & epoch.
        (* fire_when_enabled *)
        rule set_current_timeslot (running[i] == 1 && is_schedule_set[i] == 1);
        
            if (clock_within_timeslot[i] == 0)        // New timeslot
                begin
                    // Set neighbor using current timeslot value.
                    ServerIndex d = schedule_table[host_index[i]]
                                [current_phase[i]][current_timeslot[i]];
                    current_neighbor[i] <= d;

                    num_of_time_slots_used_reg[i] <= num_of_time_slots_used_reg[i] + 1;

                    if (verbose)
                        $display("[SCHED %d] ---------------- New time slot = %d. Set phase = %d timeslot = %d nbr = %d at time = %d ----------------",
                            host_index[i], num_of_time_slots_used_reg[i], current_phase[i], current_timeslot[i], d, current_time);

                    // Choose whether to send local or remote cells in this timeslot.
                    choose_buffer_to_send_from_fifo[i][current_phase[i]][current_timeslot[i]].enq(?);
                end
            
            // Timelsot ends after current cycle.
            if (clock_within_timeslot[i] == timeslot_len - 1)
            begin
                clock_within_timeslot[i] <= 0;

                // Updates timeslot, phase, and epoch values for next clock cycle.
                current_timeslot[i] <= (current_timeslot[i] + 1)
                        % fromInteger(valueof(PHASE_SIZE));
                // New phase - check old value of current_timeslot.
                if (current_timeslot[i] == fromInteger(valueof(PHASE_SIZE)-1)) 
                begin                  
                    current_phase[i] <= (current_phase[i] + 1) 
                        % fromInteger(valueof(NUM_OF_PHASES));
                    // New epoch - check old value of phase.
                    if (current_phase[i] == fromInteger(valueof(NUM_OF_PHASES)-1))                      
                        current_epoch[i] <= current_epoch[i] + 1;
                end
            end
            else
                clock_within_timeslot[i] <= clock_within_timeslot[i] + 1;
        endrule
    end

/*------------------------------------------------------------------------------*/

                                /* Tx Path */

/*------------------------------------------------------------------------------*/

    // BRAM to store cells to forward.
    // Store fwd cells by phase instead of nbr.

    `ifdef LIMIT_ACTIVE_BUCKETS
        // bitmap to store which bucket indices are free.
        Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(NUM_ACTIVE_BUCKETS)))
            bucket_free_bitmap <- replicateM(mkReg(fromInteger(valueof(BUCKET_BITMAP_ALL_FREE))));
        
        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_ACTIVE_BUCKETS, Reg#(Bit#(4)))) 
            num_pending_pkts <- replicateM(replicateM(mkReg(0)));
   `endif

    `ifndef DEBUG_WITHOUT_BUFFERS
        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(
            `ifdef LIMIT_ACTIVE_BUCKETS
                NUM_ACTIVE_BUCKETS,
            `else
                NUM_FWD_TOKEN_BUCKETS,
            `endif
            RingBuffer#(ReadReqType, ReadResType, WriteReqType))))
                spray_buffer <- replicateM(replicateM(replicateM(
                    mkRingBuffer(valueof(PHASE_SIZE), valueof(CELL_SIZE)))));

        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(
            `ifdef LIMIT_ACTIVE_BUCKETS
                NUM_ACTIVE_BUCKETS,
            `else
                NUM_DIRECT_TOKEN_BUCKETS,
            `endif
            RingBuffer#(ReadReqType, ReadResType, WriteReqType))))
                fwd_buffer <- replicateM(replicateM(replicateM(
                    mkRingBuffer(valueof(PHASE_SIZE), valueof(CELL_SIZE)))));
    `else
        Integer num_pkts_per_cell = valueof(CELL_SIZE) / valueof(BUS_WIDTH);

        Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(6))) curr_tx_pkt_idx <- replicateM(mkReg(0));
        Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(6))) rem_fwd_req <- replicateM(mkReg(0));
        Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(6))) rem_spray_req <- replicateM(mkReg(0));
        
        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(
            `ifdef LIMIT_ACTIVE_BUCKETS
                NUM_ACTIVE_BUCKETS,
            `else
                NUM_FWD_TOKEN_BUCKETS,
            `endif
            FIFOF#(Bit#(HEADER_SIZE))))) spray_buffer <- replicateM(
                replicateM(replicateM(mkSizedFIFOF(valueof(PHASE_SIZE) * num_pkts_per_cell))));

        Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(
            `ifdef LIMIT_ACTIVE_BUCKETS
                NUM_ACTIVE_BUCKETS,
            `else
                NUM_DIRECT_TOKEN_BUCKETS,
            `endif
            FIFOF#(Bit#(HEADER_SIZE))))) fwd_buffer <- replicateM(
                replicateM(replicateM(mkSizedFIFOF(valueof(EPOCH_SIZE) * num_pkts_per_cell))));
        
    `endif
    
    // Matrix to store available tokens per nbr, bucket. Initially,
    // each bucket has 1 token each.
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, 
        Vector#(
            `ifdef LIMIT_ACTIVE_BUCKETS
                NUM_ACTIVE_BUCKETS,
            `else
                NUM_TOKEN_BUCKETS,
            `endif 
            Reg#(Bit#(TOKEN_COUNT_SIZE)))))) tkn_count 
            <- replicateM(replicateM(replicateM(replicateM(mkReg(1)))));

    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(8)))
        total_num_fwd_cells <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, 
        Reg#(Bit#(3))))) num_fwd_cells <- replicateM(replicateM(replicateM(mkReg(0))));

    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, 
        PIEOQueue))) fwd_pieo <- replicateM(replicateM(replicateM(mkPIEOQueue(0))));

    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, 
        Reg#(Bit#(1))))) pieo_init_done <- replicateM(replicateM(replicateM(mkReg(0))));
    
    
    // Initialize PIEO by resetting queues
    // TODO: Need some way to make sure this reset is over before we start scheduler ops.
    for(Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        for (Integer j = 0; j < valueof(NUM_OF_PHASES); j = j + 1)
            begin
            for (Integer k = 0; k < valueof(PHASE_SIZE); k = k + 1)
                begin
                rule test_pieo_prep(pieo_init_done[i][j][k] == 0);
                    pieo_init_done[i][j][k] <= 1;
                    fwd_pieo[i][j][k].reset_queue();
                endrule
            end
        end
    end

    // State to determine next cell to send.
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(BufType))
        buffer_to_send_from <- replicateM(mkReg(HOST));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(BUCKET_IDX_BITS)))
        tx_fwd_buffer_idx <- replicateM(mkReg(0));
        
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex))
        host_flow_to_send <- replicateM(mkReg(0));
    `ifndef DEBUG_WITHOUT_BUFFERS
    Vector#(NUM_OF_ALTERA_PORTS, FIFO#(ReadResType))
        cell_to_send_fifo <- replicateM(mkSizedFIFO(2));
    `else
    Vector#(NUM_OF_ALTERA_PORTS, FIFO#(Bit#(HEADER_SIZE)))
        cell_to_send_fifo <- replicateM(mkPipelineFIFO);
    `endif
    
    // Header for outgoing cells. 
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(HEADER_SIZE)))
        curr_header <- replicateM(mkReg(0));

    // TODO: Change to FIFOF and check for notFull when enqueuing.
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE, FIFO#(Bit#(BUCKET_IDX_BITS)))))
        pending_token <- replicateM(replicateM(replicateM(mkSizedFIFO(3))));

    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(BUCKET_IDX_BITS))) 
        token_to_send <- replicateM(mkReg(fromInteger(valueof(FINAL_DST_BUCKET_IDX))));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(BUCKET_IDX_BITS))) 
        token_to_send_2 <- replicateM(mkReg(fromInteger(valueof(FINAL_DST_BUCKET_IDX))));

    // Tx rules for each port.
    for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        // Tx rules for each destination node. At most one req_*_cell rule gets fired per timeslot,
        // depending on buffer_to_send_from type. These rules only get fired on the seventh clock cycle
        // of each timeslot. It is obvious that these should only fire on a single clock cycle of the timeslot.
        // This is the 7th cycle because - 1 cycle for set_current_timeslot, 1 for choose_buffer_to_send_from
        // 3 for dequeue to execute, 1 for choose_buffer_to_send_from_2
        // TODO: Why not just have signal FIFOs to fire these rules, instead of checking clock_within_timeslot ??
        
        // A rule to get local cells for each possible destination, when sending local traffic.
        for (Integer j = 0; j < valueof(NUM_OF_SERVERS); j = j + 1)
        begin
            rule request_local_cell (running[i] == 1 && clock_within_timeslot[i] == 7
                            && buffer_to_send_from[i] == HOST
                            && host_flow_to_send[i] == fromInteger(j));        
                cell_generator[i].host_cell_req.put(fromInteger(j));
            endrule
        end

        // This rule may multiple times, as for each host cell request, the generator responds with
        // packets of size BUS_WIDTH. So, each request triggers this rule CELL_SIZE/BUS_WIDTH times.
        // Same for FWD/SPRAY cell requests.
        // TODO: Should we check that all packets of the cell are transmitted to the same (correct) neighbor.
        rule get_local_cell;
            let d <- cell_generator[i].host_cell_res.get;          
            `ifndef DEBUG_WITHOUT_BUFFERS
            cell_to_send_fifo[i].enq(d);
            `else
            Integer s = valueof(BUS_WIDTH) - 1;
            Integer e = valueof(BUS_WIDTH) - valueof(HEADER_SIZE);
            Bit#(HEADER_SIZE) h = d.data.payload[s:e];
            cell_to_send_fifo[i].enq(h);
            `endif
        endrule
        
        // A rule to get remote cells for each possible neighbor based on timeslot & phase, when sending remote traffic.
        for (Integer j = 0; j < valueof(NUM_OF_PHASES); j = j + 1)
        begin
            // Get cell to send from remote buffer of current neighbor
            // and the bucket selected via PIEO.
            rule request_fwd_cell (running[i] == 1 && clock_within_timeslot[i] == 7
                            && (buffer_to_send_from[i] == FWD || buffer_to_send_from[i] == SPRAY)
                            && current_phase[i] == fromInteger(j));              
                let bkt_idx = tx_fwd_buffer_idx[i];
                if (buffer_to_send_from[i] == FWD) 
                    `ifndef DEBUG_WITHOUT_BUFFERS
                    fwd_buffer[i][j][bkt_idx].read_request.put(makeReadReq(READ));
                    `else
                    rem_fwd_req[i] <= fromInteger(num_pkts_per_cell);
                    `endif
                else
                    `ifndef DEBUG_WITHOUT_BUFFERS
                    spray_buffer[i][j][bkt_idx].read_request.put(makeReadReq(READ));
                    `else
                    rem_spray_req[i] <= fromInteger(num_pkts_per_cell);
                    `endif
                // TODO: do we need to fix this conflict with receive_cell?    
                `ifdef LIMIT_ACTIVE_BUCKETS
                    num_pending_pkts[i][bkt_idx] <= num_pending_pkts[i][bkt_idx] - 1;
                `endif
            endrule

            // For each neighbor & bucket, if a cell is read, prepare to send it.
            `ifdef LIMIT_ACTIVE_BUCKETS
            for(Integer l = 0; l < valueof(NUM_ACTIVE_BUCKETS); l = l + 1)
            `else
            for(Integer l = 0; l < valueof(NUM_FWD_TOKEN_BUCKETS); l = l + 1)
            `endif
            begin
                `ifndef DEBUG_WITHOUT_BUFFERS
                rule get_spray_cell;
                    let d <- spray_buffer[i][j][l].read_response.get;

                `else
                rule get_spray_cell (rem_spray_req[i] > 0
                    && current_phase[i] == fromInteger(j)
                    && tx_fwd_buffer_idx[i] == fromInteger(l));
                    let d <- toGet(spray_buffer[i][j][l]).get;
                    rem_spray_req[i] <= rem_spray_req[i] - 1;

                `endif
                    cell_to_send_fifo[i].enq(d);
                    
                    `ifdef DEBUG_WITHOUT_BUFFERS if (rem_spray_req[i] == 1)
                    `else if (d.data.eop == 1)
                    `endif
                    begin
                        total_num_fwd_cells[i] <= total_num_fwd_cells[i] - 1;
                        num_fwd_cells[i][j][current_timeslot[i]] <= 
                            num_fwd_cells[i][j][current_timeslot[i]] - 1;
                    end
                endrule
            end

            `ifdef LIMIT_ACTIVE_BUCKETS
            for(Integer l = 0; l < valueof(NUM_ACTIVE_BUCKETS); l = l + 1)
            `else
            for(Integer l = 0; l < valueof(NUM_DIRECT_TOKEN_BUCKETS); l = l + 1)
            `endif
            begin
                `ifndef DEBUG_WITHOUT_BUFFERS
                rule get_fwd_cell;
                    let d <- fwd_buffer[i][j][l].read_response.get;

                `else
                rule get_fwd_cell (rem_fwd_req[i] > 0
                    && current_phase[i] == fromInteger(j)
                    && tx_fwd_buffer_idx[i] == fromInteger(l));
                    let d <- toGet(fwd_buffer[i][j][l]).get;
                    rem_fwd_req[i] <= rem_fwd_req[i] - 1;

                `endif
                    cell_to_send_fifo[i].enq(d);

                    `ifdef DEBUG_WITHOUT_BUFFERS if (rem_fwd_req[i] == 1)
                    `else if (d.data.eop == 1)
                    `endif
                    begin
                        total_num_fwd_cells[i] <= total_num_fwd_cells[i] - 1;
                        num_fwd_cells[i][j][current_timeslot[i]] <= 
                            num_fwd_cells[i][j][current_timeslot[i]] - 1;
                    end
                endrule
            end

            for(Integer k = 0; k < valueof(PHASE_SIZE); k = k + 1)
            begin
                rule get_pending_token (running[i] == 1 && current_phase[i] == fromInteger(j)
                                        && current_timeslot[i] == fromInteger(k) &&
                                        clock_within_timeslot[i] == 7);
                    let bkt <- toGet(pending_token[i][j][k]).get;
                    token_to_send[i] <= bkt;
                    if (verbose && i == 7)
                        $display("[SCHED %d] getting first pending token bkt %d time=%d", 
                            i, bkt, current_time);
                endrule

                // rule get_pending_token_2 (running[i] == 1 && current_phase[i] == fromInteger(j)
                //                         && current_timeslot[i] == fromInteger(k) &&
                //                         clock_within_timeslot[i] == 8);
                //     let bkt <- toGet(pending_token[i][j][k]).get;
                //     token_to_send_2[i] <= bkt;
                //     if (verbose && i == 7)
                //         $display("[SCHED %d] getting second pending token bkt %d time=%d", 
                //             i, bkt, current_time);
                // endrule
            end
        end

        // Stat collection for buffer lengths
        rule get_max_buffer_len (running[i] == 1);
            if(total_num_fwd_cells[i] > max_fwd_buffer_length_reg[i])
                max_fwd_buffer_length_reg[i] <= extend(total_num_fwd_cells[i]);
        endrule

        rule req_dummy_cell (running[i] == 1 && clock_within_timeslot[i] == 7
                        && buffer_to_send_from[i] == DUMMY);
            cell_generator[i].dummy_cell_req.put(?);
        endrule

        rule get_dummy_cell;
            let d <- cell_generator[i].dummy_cell_res.get;
            `ifndef DEBUG_WITHOUT_BUFFERS
                cell_to_send_fifo[i].enq(d);
            `else
                // TODO: Debug w/o buffers assumes single block per cell
                Integer s = valueof(BUS_WIDTH) - 1;
                Integer e = valueof(BUS_WIDTH) - valueof(HEADER_SIZE);
                Bit#(HEADER_SIZE) h = d.data.payload[s:e];
                cell_to_send_fifo[i].enq(h);
            `endif
        endrule

        // Rule to actually send by forwarding cell to lower layers. 
        // Only fires once we enq to cell_to_send_fifo. 
        // NOTE: It takes one cycle to transmit each BUS_WIDTH (512) block. 
        // So, to make best use of our timeslot, we should use cell_size close
        // to # of transmit cyles (timeslot - processing overhead)
        rule send_cell (running[i] == 1);
            let d <- toGet(cell_to_send_fifo[i]).get;
            
            Bit#(HEADER_SIZE) h = curr_header[i];
            `ifndef DEBUG_WITHOUT_BUFFERS
            if (d.data.sop == 1)
            begin
                Integer s = valueof(BUS_WIDTH) - 1;
                Integer e = valueof(BUS_WIDTH) - valueof(HEADER_SIZE);
                h = d.data.payload[s:e];
            `else
            if (curr_tx_pkt_idx[i] == 0)
            begin
                h = d;
            `endif
                // Set mac layer information in header, once per cell sent.
                h[valueof(HDR_SRC_MAC_S):valueof(HDR_SRC_MAC_E)] = host_index[i];         // src_mac
                h[valueof(HDR_DST_MAC_S):valueof(HDR_DST_MAC_E)] = current_neighbor[i];   // dst_mac
                h[valueof(HDR_SRC_PHASE_S):valueof(HDR_SRC_PHASE_E)] = current_phase[i];  // src_mac_phase
                // TODO: Was Vishal doing this via a | operation:   
                // Bit#(HEADER_SIZE) x = {host_index[i], current_neighbor[i], '0};
                // h = d.data.payload[s:e] | x;

                // TODO: Add second token processing!
                Bit#(TOKEN_SIZE) dummy_tkn = '1;    
                Bit#(BUCKET_IDX_BITS) token_bkt = token_to_send[i];
                if (token_bkt != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                begin
                    Token tkn = get_token_from_bucket_idx(token_bkt);
                    h[valueof(HDR_TOKEN_1_S):valueof(HDR_TOKEN_1_E)] = pack(tkn);
                    if (verbose)
                        $display("[SCHED %d] Attaching token for bkt %d, %d (# %d)", 
                            host_index[i], tkn.dst_ip, tkn.remaining_spraying_hops, token_bkt);
                    token_to_send[i] <= fromInteger(valueof(FINAL_DST_BUCKET_IDX));
                end
                else h[valueof(HDR_TOKEN_1_S):valueof(HDR_TOKEN_1_E)] = dummy_tkn;
                curr_header[i] <= h;
            end

            ServerIndex dst_ip = h[valueof(HDR_DST_IP_S):valueof(HDR_DST_IP_E)];
            Bit#(14) seq_num = h[valueof(HDR_SEQ_NUM_S):valueof(HDR_SEQ_NUM_E)];

            // NOTE: Change this if header size changes!
            // TODO: hard-coded index
            `ifdef DEBUG_WITHOUT_BUFFERS
                RingBufferDataT data = defaultValue;
                data.sop = (curr_tx_pkt_idx[i] == 0) ? 1 : 0;
                data.eop = (curr_tx_pkt_idx[i] == fromInteger(num_pkts_per_cell-1)) ? 1 : 0;
                curr_tx_pkt_idx[i] <= (curr_tx_pkt_idx[i] + 1) % fromInteger(num_pkts_per_cell);    
            `else
                RingBufferDataT data = d.data;
            `endif
            if (data.eop == 1)
                data.payload = {h, h, h, h, h, current_time};
            else
                data.payload = {h, h, h, h, h, h[87:16]};

            // Put to MAC interface.
            mac.mac_tx_write_req[i].put(data);

            if (verbose)
                $display("[SCHED %d] Sending cell dst = %d seq_num=%d sop = %d eop = %d to neighbor %d at time %d (mac time=%d)",
                    host_index[i], dst_ip, seq_num ,data.sop, data.eop, current_neighbor[i], current_time, mac.getTxClock());
        endrule
    end

/*------------------------------------------------------------------------------*/

                                /* Rx Path */

/*------------------------------------------------------------------------------*/

    // TODO: Need a proper priority encoder here!!
    function Bit#(BUCKET_IDX_BITS) get_first_one_index(Bit#(NUM_ACTIVE_BUCKETS) bitmap);
        Bit#(NUM_ACTIVE_BUCKETS) x = bitmap;
        Bit#(BUCKET_IDX_BITS) idx = fromInteger(valueof(FINAL_DST_BUCKET_IDX));
        for(Integer i = 0; i< valueof(NUM_ACTIVE_BUCKETS); i = i + 1)
        begin
            if ((x % 2 == 1) && idx == fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                idx = fromInteger(i);
            x = x >> 1;
        end
        get_first_one_index = idx;
    endfunction

    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex))
        curr_src_mac <- replicateM(mkReg(fromInteger(valueof(NUM_OF_SERVERS))));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex))
        curr_dst_mac <- replicateM(mkReg(fromInteger(valueof(NUM_OF_SERVERS))));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex))
        curr_src_ip <- replicateM(mkReg(fromInteger(valueof(NUM_OF_SERVERS))));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(ServerIndex))
        curr_dst_ip <- replicateM(mkReg(fromInteger(valueof(NUM_OF_SERVERS))));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(1)))
        curr_dummy_bit <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(HEADER_SIZE)))
        curr_rx_header <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(1)))
        curr_corrupted_cell <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(16)))
        curr_cell_size <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Phase)) 
        curr_remaining_spray_hops <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Phase)) 
        curr_src_mac_phase <- replicateM(mkReg(0));    
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Phase)) 
        curr_rx_next_hop_phase <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Coordinate)) 
        curr_rx_next_hop_timeslot <- replicateM(mkReg(0));
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(BUCKET_IDX_BITS))) 
        curr_rx_bucket_idx <- replicateM(mkReg(0));
    `ifndef LIMIT_ACTIVE_BUCKETS
        Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(BUCKET_IDX_BITS))) 
            curr_rx_buffer_idx <- replicateM(mkReg(0));
    `endif
    Vector#(NUM_OF_ALTERA_PORTS, Reg#(Bit#(1))) 
        curr_rx_bkt_found <- replicateM(mkReg(0));

    
    // TODO: Set bits for num tokens.
    // TODO: Two tokens for same bucket opt.
    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE,
        FIFO#(Bit#(BUCKET_IDX_BITS))))) update_tokens_fifo <- replicateM(replicateM(replicateM(mkFIFO))); 
    Vector#(NUM_OF_ALTERA_PORTS, FIFO#(Bit#(BUCKET_IDX_BITS)))
        deallocate_bkt_fifo <- replicateM(mkFIFO); 

    Vector#(NUM_OF_ALTERA_PORTS, Vector#(NUM_OF_PHASES, Vector#(PHASE_SIZE,
        FIFO#(Bit#(BUCKET_IDX_BITS))))) allocate_host_bkt_fifo <- replicateM(replicateM(replicateM(mkBypassFIFO)));
    // PIEOCurrentTime pieo_current_time_in;

    // Module to pick spraying hops.
    // RandomHopGenerator rng_hop <-mkRandomHopGenerator;

    for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        rule receive_cell (running[i] == 1);
            // Get from MAC interface
            let d <- mac.mac_rx_read_res[i].get;

            ServerIndex src_mac = curr_src_mac[i];
            ServerIndex dst_mac = curr_dst_mac[i];
            ServerIndex src_ip = curr_src_ip[i];
            ServerIndex dst_ip = curr_dst_ip[i];
            Phase src_mac_phase = curr_src_mac_phase[i];
            Phase remaining_spraying_hops = curr_remaining_spray_hops[i];
            Bit#(1) dummy_bit = curr_dummy_bit[i];

            `ifdef LIMIT_ACTIVE_BUCKETS
                Bit#(BUCKET_IDX_BITS) replenished_bkt = fromInteger(valueof(FINAL_DST_BUCKET_IDX));
            `endif

            Integer src_mac_phase_int = phase_to_int(curr_src_mac_phase[i]);

            Bit#(HEADER_SIZE) hd = curr_rx_header[i];

            Bit#(1) corrupted_cell = curr_corrupted_cell[i];

            // All the indicies assume BUS_WIDTH of 512; change them if you change
            // BUS_WIDTH

            Bit#(16) cell_size_cnt = curr_cell_size[i];

            if (d.sop == 1)
            begin
                // TODO: hard-coded indices
                // For first chunk (BUS_WIDTH size) of cell, extract header
                hd = d.payload[511:424];
                curr_rx_header[i] <= hd;

                corrupted_cell = 0;
                src_mac = hd[valueof(HDR_SRC_MAC_S): valueof(HDR_SRC_MAC_E)];
                dst_mac = hd[valueof(HDR_DST_MAC_S): valueof(HDR_DST_MAC_E)];
                src_ip = hd[valueof(HDR_SRC_IP_S): valueof(HDR_SRC_IP_E)];
                dst_ip = hd[valueof(HDR_DST_IP_S): valueof(HDR_DST_IP_E)];
                src_mac_phase = hd[valueof(HDR_SRC_PHASE_S): valueof(HDR_SRC_PHASE_E)];
                remaining_spraying_hops = hd[valueof(HDR_SPRAY_HOPS_S):valueof(HDR_SPRAY_HOPS_E)];
                dummy_bit = hd[valueof(HDR_DUMMY_BIT)];

                src_mac_phase_int = phase_to_int(src_mac_phase);

                curr_src_mac[i] <= src_mac;
                curr_dst_mac[i] <= dst_mac;
                curr_src_ip[i] <= src_ip;
                curr_dst_ip[i] <= dst_ip;
                curr_src_mac_phase[i] <= src_mac_phase;
                curr_remaining_spray_hops[i] <= remaining_spraying_hops;
                curr_dummy_bit[i] <= dummy_bit;

                if (dst_mac != host_index[i])
                begin
                    if (verbose)
                        $display("[SCHED %d] GOT WRONG PKT: src_mac=%d dst_mac=%d src_mac_phase=%d", 
                                    host_index[i], src_mac, dst_mac, src_mac_phase);
                    corrupted_cell = 1;
                    num_of_wrong_dst_pkt_received_reg[i]
                        <= num_of_wrong_dst_pkt_received_reg[i] + 1;
                end

                cell_size_cnt = fromInteger(valueof(BUS_WIDTH));
                curr_cell_size[i] <= cell_size_cnt;
            end
            else
            begin
                cell_size_cnt = cell_size_cnt + fromInteger(valueof(BUS_WIDTH));
                curr_cell_size[i] <= cell_size_cnt;
            end

            // ------------- Check for corruption + deliver, completely taken from Shoal ------------
            // TODO: Fix all hard-coded indices, define these in RingBufferTypes? 

            // TODO: Assumes header size of 64 and BUS_WIDTH of 512, fix for different values of these.
            // Last header chunk is of size BUS_WIDTH % header size.
            // hard-coded header size
            Bit#(BUS_WIDTH) c = {hd, hd, hd, hd, hd, hd[87:16]};

            if (corrupted_cell == 0)
            begin
                if (d.eop == 0)
                begin
                // For every packet except last, cell is set to repeated header blocks (in tx path).
                // If this doesn't match what is receied, packet is corrupted.
                    if (d.payload != c)
                    begin
                        corrupted_cell = 1;
                        // num_of_corrupted_pkt_received_reg[i]
                        //     <= num_of_corrupted_pkt_received_reg[i] + 1;
                    end
                end
                else if (d.eop == 1)
                begin
                // For last packet, only the last (BUS_WIDTH % header size) bits will be set to time of sending instead of header, check everything else.
                // TODO: hard-coded indices.
                    if (cell_size_cnt != fromInteger(valueof(CELL_SIZE))
                        || d.payload[511:72] != c[511:72])
                    begin
                        corrupted_cell = 1;
                        if(verbose)
                            $display("[SCHED %d] GOT CORRUPTED PKT at time=%d", host_index[i], current_time);
                        // num_of_corrupted_pkt_received_reg[i]
                        //     <= num_of_corrupted_pkt_received_reg[i] + 1;
                    end

                    if (corrupted_cell == 0 && dummy_bit == 0)
                    begin
                        // Collect latency stats for the cell for a single hop at MAC layer (time_recvd - time_sent).
                        // This one-way delay could be skewed for different clocks at different FPGA boards without clock sync. 
                        // TODO: Shoal was counting corrupted packets towards latency stats as well? Also, only one data point, not agg.
                        // TODO: hard-coded indices.
                        Bit#(72) send_time = d.payload[71:0];
                        Bit#(72) latency = current_time - send_time;
                        if (send_time != 0 && latency_reg[i] == 0)
                            latency_reg[i] <= latency;

                        if (verbose)
                        begin
                            // hard-code indices.
                            Bit#(14) seq_num = hd[valueof(HDR_SEQ_NUM_S):valueof(HDR_SEQ_NUM_E)];
                            $display("[SCHED %d] Received cell with src_mac=%d; src_ip=%d; dst_ip=%d; src_mac_phase=%d; seq_num=%d latency=%d cycles at phase=%d; send_time=%d; recv_time=%d",
                                        host_index[i], src_mac, src_ip, dst_ip, src_mac_phase, seq_num, latency, current_phase[i], send_time, current_time);
                        end
                        if (dst_ip == host_index[i])
                            num_of_host_pkt_received_reg[i]
                                <= num_of_host_pkt_received_reg[i] + 1;
                        // else
                        //     num_of_fwd_pkt_received_reg[i]
                        //         <= num_of_fwd_pkt_received_reg[i] + 1;
                    end
                end
            end

            if (d.sop == 1 && corrupted_cell == 0)
            begin
                // Parse tokens and update count.
                // TODO: hard-coded indices..
                Token tkn1;
                tkn1.dst_ip = hd[valueof(HDR_TOKEN_1_S): valueof(HDR_TOKEN_1_S)-8];
                tkn1.remaining_spraying_hops = hd[valueof(HDR_TOKEN_1_S)-9: valueof(HDR_TOKEN_1_E)];
                ServerIndex null_idx = '1;
                if (tkn1.dst_ip != null_idx)
                begin
                    if (verbose)
                        $display("[SCHED %d] got token for bkt %d,%d", 
                            host_index[i], tkn1.dst_ip, tkn1.remaining_spraying_hops);
                    Coordinate c_ = get_coordinate(src_mac, src_mac_phase_int);
                    Coordinate slot = get_timeslot_with_matching_coordinate(host_index[i], c_, src_mac_phase_int);

                    // TODO: add second token processing.
                    let tkn_bkt_idx = get_fwd_bucket_from_tkn(tkn1);
                    `ifdef LIMIT_ACTIVE_BUCKETS
                        Bit#(BUCKET_IDX_BITS) bkt_idx = token_bkt_map[i][tkn_bkt_idx];
                        
                        // Increment oken count.
                        update_tokens_fifo[i][src_mac_phase][slot].enq(bkt_idx);

                        // TODO: Use max_tokens instead of 1 here.
                        // Check if all tokens are replenished for this bucket.
                        Bit#(4) num_pending_tokens_ = 0;
                        for (Integer j = 0; j < valueof(NUM_OF_PHASES); j = j + 1)
                            for (Integer k = 0; k < valueof(PHASE_SIZE); k = k + 1)
                                num_pending_tokens_ = num_pending_tokens_ +
                                             extend(1 - tkn_count[i][j][k][bkt_idx]);
                        num_pending_tokens_ = num_pending_tokens_ - 1;
                        
                        // If all pending tokens for a bucket have been received, we might
                        // de-allocate it (later in this rule).
                        if (num_pending_pkts[i][bkt_idx] == 0 && num_pending_tokens_ == 0)
                            replenished_bkt = bkt_idx;
                    `else
                        update_tokens_fifo[i][src_mac_phase][slot].enq(tkn_bkt_idx);
                    `endif
                end
            end

            curr_corrupted_cell[i] <= corrupted_cell;

            // --------------------------- Forwarding Logic ------------------------------
            // TODO: Will probably have to move some of this to a different rule
            // and define conflict behaviour with Tx path rules.
            if (corrupted_cell == 0 && dummy_bit == 0 && dst_ip != host_index[i])
            begin

                Phase next_hop_phase = curr_rx_next_hop_phase[i];
                Coordinate next_hop_timeslot = curr_rx_next_hop_timeslot[i];
                Phase rem_spraying_hops_recvd = remaining_spraying_hops;

                Bit#(BUCKET_IDX_BITS) bucket_idx = curr_rx_bucket_idx[i];
                `ifndef LIMIT_ACTIVE_BUCKETS
                    Bit#(BUCKET_IDX_BITS) buffer_idx = curr_rx_buffer_idx[i];
                `else
                    Bit#(BUCKET_IDX_BITS) buffer_idx = bucket_idx;
                `endif
                Bit#(1) bkt_found = curr_rx_bkt_found[i];

                if (d.sop == 1)
                begin
                    next_hop_phase = 0;
                    next_hop_timeslot = 0;
                    ServerIndex next_hop = fromInteger(valueof(NUM_OF_SERVERS) + 1);
                    if (remaining_spraying_hops > 0)
                    begin
                        next_hop_phase = (src_mac_phase + 1) % fromInteger(valueof(NUM_OF_PHASES));
                        
                        // Select random hop if there is more than one hop possible.
                        // Bit#(3) spray_slot = 0;                    
                        // if (valueof(PHASE_SIZE) > 1)
                        // begin  
                        //     spray_slot <- rng_hop.get;
                        //     if(verbose) $display("Random hop selected: %d", spray_slot);
                        // end
                        // next_hop = schedule_table[i][next_hop_phase][spray_slot];

                        // Spray short.
                        Bit#(3) min_buffer_len = fromInteger(valueof(PHASE_SIZE) + 1);
                        Token bucket;
                        bucket.dst_ip = dst_ip;
                        bucket.remaining_spraying_hops = remaining_spraying_hops;
                        for(Integer j = 0; j < valueof(PHASE_SIZE); j = j + 1)
                        begin
                            ServerIndex nbr = schedule_table[i][next_hop_phase][j];
                            Bit#(3) len = num_fwd_cells[i][next_hop_phase][j];
                            if(min_buffer_len > len)
                            begin
                                min_buffer_len = len;
                                next_hop = nbr;
                                next_hop_timeslot = fromInteger(j);
                            end
                        end
                        
                        rem_spraying_hops_recvd = remaining_spraying_hops;
                        remaining_spraying_hops = remaining_spraying_hops - 1;
                        d.payload[458:456] = remaining_spraying_hops;           // TODO: hard-coded index!
                        if (verbose)
                            $display("[SCHED %d] Rx: Fwd cell to spray_hop=%d (min_buf=%d) remaining spraying hops=%d, dst=%d on phase %d at time=%d", 
                                host_index[i], next_hop, min_buffer_len, remaining_spraying_hops, dst_ip, next_hop_phase, current_time);

                    end
                    else
                    begin
                        // Find next phase where my coordinates don't match with the destination.
                        Integer next_phase = src_mac_phase_int;
                        for (Integer phase = 0; phase < valueof(NUM_OF_PHASES); phase = phase + 1)
                        begin
                            next_phase = (next_phase + 1) % valueof(NUM_OF_PHASES);
                            Coordinate dst_coord = get_coordinate(dst_ip, next_phase);
                            Coordinate my_coord = get_coordinate(host_index[i], next_phase);
                            // If next hop not yet found, and phase coordinate doesn't match me, only then set.
                            if (next_hop == fromInteger(valueof(NUM_OF_SERVERS) + 1) &&
                                dst_coord != my_coord) 
                            begin
                                next_hop_phase = fromInteger(next_phase);
                                next_hop_timeslot = get_timeslot_with_matching_coordinate(host_index[i], dst_coord, next_phase);
                                next_hop = schedule_table[i][next_hop_phase][next_hop_timeslot];
                                if (verbose)
                                begin
                                    let next_hop_coord = get_coordinate(next_hop, next_phase);
                                    $display("[SCHED %d] Rx: Fwd cell to direct_hop=%d on phase=%d for dst=%d, at time = %d", 
                                                host_index[i], next_hop, next_phase, dst_ip, current_time);
                                end
                            end
                        end
                    end

                    // Get send token idx for this packet, and the index within
                    // the fwd buffers. 
                    bucket_idx = get_fwd_bucket_idx(dst_ip, remaining_spraying_hops);
                    buffer_idx = bucket_idx;
                    if (rem_spraying_hops_recvd == 0)
                        buffer_idx = get_direct_buffer_idx_from_bucket(bucket_idx);

                    bkt_found = 1;
                    `ifdef LIMIT_ACTIVE_BUCKETS
                        // If bucket not allocated
                        if (token_bkt_map[i][bucket_idx] == fromInteger(valueof(NUM_ACTIVE_BUCKETS)))
                        begin
                            Bit#(BUCKET_IDX_BITS) next_free_bkt = get_first_one_index(bucket_free_bitmap[i]);
                            if (next_free_bkt != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                            begin
                                buffer_idx = next_free_bkt;
                                bucket_free_bitmap[i] <= bucket_free_bitmap[i] - (1 << next_free_bkt);
                                $display("[SCHED %d] Allocated bucket %d to tkn %d, bucket bitmap now: %d",
                                        host_index[i], buffer_idx, bucket_idx,
                                        bucket_free_bitmap[i] - (1 << next_free_bkt));
                                token_bkt_map[i][bucket_idx] <= buffer_idx;
                                bkt_token_map[i][buffer_idx] <= bucket_idx;
                            end
                            else
                            begin
                                $display("[SCHED %d] NO MORE FREE BUCKETS!", host_index[i]);
                                bkt_found = 0;
                            end
                        end
                        else buffer_idx = token_bkt_map[i][bucket_idx];
                        bucket_idx = buffer_idx;

                        if (bucket_idx != replenished_bkt && 
                            replenished_bkt != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                            deallocate_bkt_fifo[i].enq(replenished_bkt);
                    `endif

                    curr_rx_next_hop_phase[i] <= next_hop_phase;
                    curr_rx_next_hop_timeslot[i] <= next_hop_timeslot;
                    curr_rx_bucket_idx[i] <= bucket_idx;
                    `ifndef LIMIT_ACTIVE_BUCKETS
                        curr_rx_buffer_idx[i] <= buffer_idx;
                        num_pending_pkts[i][bucket_idx] <=
                            num_pending_pkts[i][bucket_idx] + 1;
                    `endif
                end

                if (bkt_found == 1)
                begin
                    Bit#(1) written_to_buffer = 0;
                    `ifndef DEBUG_WITHOUT_BUFFERS
                        if (rem_spraying_hops_recvd > 0 && !spray_buffer[i][next_hop_phase][buffer_idx].full)
                        begin
                            spray_buffer[i][next_hop_phase][buffer_idx].write_request.put
                                (makeWriteReq(d.sop, d.eop, d.payload));
                            written_to_buffer = 1;
                        end
                        else if (rem_spraying_hops_recvd == 0 && !fwd_buffer[i][next_hop_phase][buffer_idx].full)
                        begin
                            fwd_buffer[i][next_hop_phase][buffer_idx].write_request.put
                                (makeWriteReq(d.sop, d.eop, d.payload));
                            written_to_buffer = 1;
                        end
                    `else
                        if (rem_spraying_hops_recvd > 0 && spray_buffer[i][next_hop_phase][buffer_idx].notFull())
                        begin
                            Integer s = valueof(BUS_WIDTH) - 1;
                            Integer e = valueof(BUS_WIDTH) - valueof(HEADER_SIZE);
                            Bit#(HEADER_SIZE) h = d.payload[s:e];
                            spray_buffer[i][next_hop_phase][buffer_idx].enq(h);
                            written_to_buffer = 1;
                        end
                        else if (rem_spraying_hops_recvd == 0 && fwd_buffer[i][next_hop_phase][buffer_idx].notFull())
                        begin
                            Integer s = valueof(BUS_WIDTH) - 1;
                            Integer e = valueof(BUS_WIDTH) - valueof(HEADER_SIZE);
                            Bit#(HEADER_SIZE) h = d.payload[s:e];
                            fwd_buffer[i][next_hop_phase][buffer_idx].enq(h);
                            written_to_buffer = 1;
                        end
                    `endif 

                    if (written_to_buffer == 1 && d.eop == 1)
                    begin
                        if (dst_ip == schedule_table[i][next_hop_phase][next_hop_timeslot])
                            bucket_idx = fromInteger(valueOf(FINAL_DST_BUCKET_IDX));
                        PIEOElement flow;
                        flow.id = bucket_idx;
                        flow.rank = 0;
                        flow.prev_hop_phase = src_mac_phase;
                        flow.prev_hop_slot = get_timeslot_with_matching_coordinate(host_index[i], 
                                                get_coordinate(src_mac, src_mac_phase_int), src_mac_phase_int);
                        flow.rem_spraying_hops_recvd = rem_spraying_hops_recvd;
                        flow.is_spray = (rem_spraying_hops_recvd > 0) ? 1 : 0;

                        add_pending_fwd_flow[i][next_hop_phase][next_hop_timeslot].enq(flow);
                    end
                    else if (written_to_buffer == 0) 
                    begin
                        $display("[SCHED %d] FWD BUCKET FULL for nbr=%d, bucket=%d spray=%d!!", 
                            host_index[i], schedule_table[i][next_hop_phase][next_hop_timeslot], bucket_idx, (rem_spraying_hops_recvd > 0) ? 1 : 0);
                        // Check for written_to_buffer for all pkts of cell.
                        bkt_found = 0;
                    end
                end
                
                // bkt_found is bkt_found && written_to_buffer.
                if (d.sop == 1)
                    curr_rx_bkt_found[i] <= bkt_found;
            end

        endrule

        rule deallocate_bkt;
            let bkt_idx <- toGet(deallocate_bkt_fifo[i]).get;
            $display("[SCHED %d] De-allocating bucket %d at time=%d", host_index[i], bkt_idx, current_time);
            bucket_free_bitmap[i] <= bucket_free_bitmap[i] | 1 << bkt_idx;
            let tkn_bkt_idx = bkt_token_map[i][bkt_idx];
            token_bkt_map[i][tkn_bkt_idx] <= fromInteger(valueof(NUM_ACTIVE_BUCKETS));
            bkt_token_map[i][bkt_idx] <= fromInteger(valueof(NUM_TOKEN_BUCKETS));
        endrule

/*------------------------------------------------------------------------------*/

                                /* Shoal */

/*------------------------------------------------------------------------------*/

    // For each src dst pair, rule for choosing cell to send between src-dst.
    // TODO: Move all of these into per-node rules, and not per-node, per-neighbor rules
    // to reliably encode priority between rules. Anyway, these rules cannot execute together. 
        for (Integer j = 0; j < valueof(NUM_OF_PHASES); j = j + 1)
        begin
            for (Integer k = 0; k < valueof(PHASE_SIZE); k = k + 1)
            begin

                (* descending_urgency = "choose_buffer_to_send_from, update_tokens" *)
                (* descending_urgency = "choose_buffer_to_send_from_2, update_tokens" *)
                (* descending_urgency = "choose_buffer_to_send_from_2, receive_cell" *)
                (* descending_urgency = "choose_buffer_to_send_from, enq_fwd_cell" *)

                // To select local flow to send, we want to pick the next flow 
                // in round robin that has cells to send.  
                rule choose_buffer_to_send_from;
                    let dc <- toGet(choose_buffer_to_send_from_fifo[i][j][k]).get;
                    
                    // Final dst bucket is always eligible. For others (up to num_buckets - 1)
                    // check that there exist tokens AND packets.
                    PIEOCurrentTime curr_time_in = 1 << fromInteger(valueof(FINAL_DST_BUCKET_IDX));
                    `ifdef LIMIT_ACTIVE_BUCKETS
                    for(Integer bkt = 0; bkt < valueof(NUM_ACTIVE_BUCKETS); bkt = bkt + 1)
                    `else
                    for(Integer bkt = 0; bkt < valueof(NUM_FWD_TOKEN_BUCKETS); bkt = bkt + 1)
                    `endif
                    begin
                        if (tkn_count[i][j][k][bkt] > 0)    
                            curr_time_in = curr_time_in | 1 << fromInteger(bkt);
                    end
                    // $display("[SCHED %d] sending deq request with bitmap %d at time=%d",
                    //         host_index[i], curr_time_in, current_time);
                    fwd_pieo[i][j][k].dequeue(extend(curr_time_in));
                endrule

                // PIEO's dequeue result method is only enabled for 1 cylce after the op.
                // Need to have a method with no conflicts to be sure we get this value!
                rule get_deq_result (running[i] == 1);
                    PIEOElement x <- toGet(fwd_pieo[i][j][k].get_dequeue_result).get;
                    choose_buffer_to_send_from_2_fifo[i][j][k].enq(x);
                    $display("[SCHED %d] got deq result=%d at time=%d for nbr %d,%d", 
                                host_index[i], x.id, current_time, j, k);
                endrule

                rule choose_buffer_to_send_from_2;
                    PIEOElement x <- toGet(choose_buffer_to_send_from_2_fifo[i][j][k]).get;                    
                    if (x.id != fromInteger(valueof(PIEO_NULL_ID)))
                    begin
                        Bit#(BUCKET_IDX_BITS) bkt_idx = x.id;
                        if (x.id == fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                        begin
                            bkt_idx = get_fwd_bucket_idx(schedule_table[i][j][k], 
                                               x.rem_spraying_hops_recvd - extend(x.is_spray));
                            `ifdef LIMIT_ACTIVE_BUCKETS bkt_idx = token_bkt_map[i][bkt_idx]; `endif
                        end

                        if (x.is_spray == 1)
                        begin
                            buffer_to_send_from[i] <= SPRAY;
                            tx_fwd_buffer_idx[i] <= bkt_idx;
                            if (verbose)
                                $display("[SCHED %d] chose to send SPRAY cell from bucket %d (%d), prev_hop = %d at time=%d", 
                                    host_index[i], bkt_idx, x.id, schedule_table[i][x.prev_hop_phase][x.prev_hop_slot], current_time);
                        end
                        else
                        begin
                            buffer_to_send_from[i] <= FWD;
                            `ifdef LIMIT_ACTIVE_BUCKETS
                                tx_fwd_buffer_idx[i] <= bkt_idx;
                            `else
                                tx_fwd_buffer_idx[i] <= get_direct_buffer_idx_from_bucket(bkt_idx);
                            `endif
                            if (verbose)
                                $display("[SCHED %d] chose to send FWD cell from bucket %d (%d), prev_hop = %d at time = %d", 
                                    host_index[i], bkt_idx, x.id, schedule_table[i][x.prev_hop_phase][x.prev_hop_slot], current_time);
                        end
                        
                        if (x.id != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                        begin
                            tkn_count[i][j][k][bkt_idx] <= tkn_count[i][j][k][bkt_idx] - 1;
                            if (verbose)
                                $display("[SCHED %d] Decreasing tokens for bkt %d nbr %d,%d at time=%d",
                                            host_index[i], bkt_idx, j, k, current_time);
                        end
                        
                        // Enqueue token to send to previous hop of chosen cell
                        Bit#(BUCKET_IDX_BITS) tkn_bkt_idx = bkt_idx;
                        `ifdef LIMIT_ACTIVE_BUCKETS
                            tkn_bkt_idx = bkt_token_map[i][bkt_idx];
                        `endif
                        Token tkn = get_token_from_bucket_idx(tkn_bkt_idx);
                        // Set number of spray hops in the token sent, based on value recvd from prev hop.
                        tkn.remaining_spraying_hops = x.rem_spraying_hops_recvd;
                        pending_token[i][x.prev_hop_phase][x.prev_hop_slot] 
                                .enq(get_fwd_bucket_from_tkn(tkn));
                    end

                    // Then send any local flows that are ready, else send dummy flows. 
                    else
                    begin                    
                        // ------- pick host flow to send -----------
                        // Initialize dst with next_local_flow_idx[i]
                        // TODO: No easy way to convert from ServerIndex to Integer?
                        // TODO: There has to be a better way to do this!! Ex: PIEO?
                        Bit#(BUCKET_IDX_BITS) host_bkt_chosen = 0;
                        ServerIndex host_flow_chosen = fromInteger(valueof(NUM_OF_SERVERS)) + 1;
                        `ifndef PERMUTATION_TRAFFIC
                        Integer dst = 0;
                        for(Integer d = 0; d < valueof(NUM_OF_SERVERS); d = d + 1)
                            if (next_local_flow_idx[i] == fromInteger(d))
                                dst = d;
                        
                        Integer base_host_bkt_idx = 1 + ((valueof(NUM_OF_PHASES)-1) 
                                                    * valueof(NUM_OF_SERVERS));
                        for(Integer d = 0; d < valueof(NUM_OF_SERVERS); d = d + 1)
                        begin
                            Integer relv_bkt = base_host_bkt_idx + dst;
                        `else
                            Integer dst = (i + 1) % valueof(NUM_OF_SERVERS);
                            Integer relv_bkt = 1 + ((valueof(NUM_OF_PHASES)-1) 
                                                    * valueof(NUM_OF_SERVERS)) + dst;
                        `endif
                            if(fromInteger(dst) == schedule_table[i][j][k])
                                relv_bkt = valueof(FINAL_DST_BUCKET_IDX);
                            
                            Bit#(1) has_tokens = 1;    
                            `ifdef LIMIT_ACTIVE_BUCKETS
                            if (token_bkt_map[i][relv_bkt] != fromInteger(valueof(NUM_ACTIVE_BUCKETS)))
                            begin
                                Bit#(BUCKET_IDX_BITS) stored_idx = token_bkt_map[i][relv_bkt];
                                if(tkn_count[i][j][k][stored_idx] == 0)
                                    has_tokens = 0;
                            end
                            `else
                                if (tkn_count[i][j][k][relv_bkt] == 0)
                                    has_tokens = 0;
                            `endif

                            if (`ifndef PERMUTATION_TRAFFIC
                                !cell_generator[i].isEmpty(dst) && dst != i
                                && host_flow_chosen == fromInteger(valueof(NUM_OF_SERVERS) + 1) &&
                                `endif
                                has_tokens == 1)
                            begin
                                host_flow_chosen = fromInteger(dst);
                                host_bkt_chosen = fromInteger(relv_bkt);
                            end
                        `ifndef PERMUTATION_TRAFFIC
                        
                            dst = (dst + 1) % valueof(NUM_OF_SERVERS);
                        end
                        `endif

                        // At least one host flow was ready to be sent, so will choose HOST.
                        if (host_flow_chosen != fromInteger(valueof(NUM_OF_SERVERS) + 1))
                        begin
                            if (verbose)
                                $display("[SCHED %d] chose to send HOST cell at time=%d.", host_index[i], current_time);
                            
                            // if (bkt_found == 1)
                            // begin
                                buffer_to_send_from[i] <= HOST;
                                host_flow_to_send[i] <= host_flow_chosen;

                                // send request to read the local cell
                                // cell_generator[i].host_cell_req.put(host_flow_chosen);

                                // Set host flow index for next turn. Check to not send to self.
                                `ifndef PERMUTATION_TRAFFIC
                                    ServerIndex next_flow_idx = (host_flow_chosen + 1) % fromInteger(valueof(NUM_OF_SERVERS));
                                    if (next_flow_idx == fromInteger(i))
                                        next_local_flow_idx[i] <= (next_flow_idx + 1) % fromInteger(valueof(NUM_OF_SERVERS));
                                    else
                                        next_local_flow_idx[i] <= next_flow_idx;
                                `endif
                            // end
                            // else buffer_to_send_from[i] <= DUMMY;

                            `ifdef LIMIT_ACTIVE_BUCKETS
                                // Allocate host cell bucket if needed.
                                // Phase phase = fromInteger(j);
                                // Coordinate slot = fromInteger(k);
                                // Bit#(12) info = {host_bkt_chosen, phase, slot};
                                if (token_bkt_map[i][host_bkt_chosen] == fromInteger(valueof(NUM_ACTIVE_BUCKETS)))
                                    allocate_host_bkt_fifo[i][j][k].enq(host_bkt_chosen);
                                else
                                begin
                                    host_bkt_chosen = token_bkt_map[i][host_bkt_chosen];
                            `endif
                            if (host_bkt_chosen != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                            begin
                                tkn_count[i][j][k][host_bkt_chosen] <= tkn_count[i][j][k][host_bkt_chosen] - 1;
                                if (verbose)
                                    $display("[SCHED %d] Decreasing tokens for bkt %d nbr %d,%d at time=%d",
                                                host_index[i], host_bkt_chosen, j, k, current_time);
                            end
                            `ifdef LIMIT_ACTIVE_BUCKETS
                            end
                            `endif
                        end

                        else
                        begin
                            if (verbose)
                                $display("[SCHED %d] chose to send dummy cell at time=%d.", host_index[i], current_time);
                            buffer_to_send_from[i] <= DUMMY;
                        end
                    end
                endrule

                // ATTN: Potential problem here when not enough free buckets.
                // If this doesn't fire or is unsuccessful in the timeslot that 
                // the HOST cell is sent, we won't decrease tokens, and update token
                // rule will erroneously process return tokens. TODO: fix
                rule allocate_host_bkt;
                    let tkn_bkt_idx <- toGet(allocate_host_bkt_fifo[i][j][k]).get;
                    // Bit#(BUCKET_IDX_BITS) tkn_bkt_idx = x[11:6];
                    // Phase j = x[5:3];
                    // Coordinate k = x[2:0];
                    
                    Bit#(BUCKET_IDX_BITS) bkt_idx = fromInteger(valueof(FINAL_DST_BUCKET_IDX));
                    `ifdef LIMIT_ACTIVE_BUCKETS
                        if (token_bkt_map[i][tkn_bkt_idx] == fromInteger(valueof(NUM_ACTIVE_BUCKETS)))
                        begin
                            Bit#(BUCKET_IDX_BITS) next_free_bkt = get_first_one_index(bucket_free_bitmap[i]);
                            if (next_free_bkt != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                            begin
                                bkt_idx = next_free_bkt;
                                bucket_free_bitmap[i] <= bucket_free_bitmap[i] - (1 << next_free_bkt); 
                                $display("[SCHED %d] Allocated bucket %d to tkn %d, bucket bitmap now: %d",
                                        host_index[i], bkt_idx, tkn_bkt_idx, 
                                        bucket_free_bitmap[i] - (1 << next_free_bkt));
                                token_bkt_map[i][tkn_bkt_idx] <= bkt_idx;
                                bkt_token_map[i][bkt_idx] <= tkn_bkt_idx;
                            end
                            else
                                $display("[SCHED %d] NO MORE FREE BUCKETS!", host_index[i]);
                        end
                        else
                            bkt_idx = token_bkt_map[i][tkn_bkt_idx];
                    `else
                        bkt_idx = tkn_bkt_idx;
                    `endif
            
                    // If SPRAY/FWD/HOST and not sending to final dest, expend a token from relevant bucket.
                    if (bkt_idx != fromInteger(valueof(FINAL_DST_BUCKET_IDX)))
                    begin
                        // if (tkn_count[i][j][k][bkt_idx] == 1)
                        // begin
                        //     PIEOCurrentTime mask = '1;
                        //     mask = mask - (1 << bkt_idx);
                        //     pieo_curr_time_in[i][j][k] = pieo_curr_time_in[i][j][k] & mask;
                        // end
                        tkn_count[i][j][k][bkt_idx] <= tkn_count[i][j][k][bkt_idx] - 1;
                        if (verbose)
                            $display("[SCHED %d] Decreasing tokens for bkt %d nbr %d,%d at time=%d",
                                        host_index[i], bkt_idx, j, k, current_time);
                    end
                endrule

                // TODO: not sure if this is really helping...
                rule enq_fwd_cell;
                    let flow <- toGet(add_pending_fwd_flow[i][j][k]).get;
                    
                    let bucket_idx = flow.id;

                    if (verbose)
                        $display("[SCHED %d] Enqueuing bucket idx %d in PIEO, fwd buffer for %d at time=%d", 
                            host_index[i], bucket_idx, schedule_table[i][j][k], current_time);
                
                    // Each idv packet is enqueued into PIEO with the bucket ID, 
                    // and we just pass current_time with eligibility info for each bucket.
                    // This way we need not call dequeue_f to update eligibility for buckets!
                    //  And we also don't need to re-enqueue buckets.
                    fwd_pieo[i][j][k].enqueue(flow);
                    total_num_fwd_cells[i] <= total_num_fwd_cells[i] + 1;
                    num_fwd_cells[i][j][k] <= 
                        num_fwd_cells[i][j][k] + 1;
                    if (num_fwd_cells[i][j][k] > 4)
                        $display("[SCHED %d] PIEO IS FULL for nbr=%d!! num_fwd_cells=%d",
                            host_index[i], schedule_table[i][j][k], num_fwd_cells[i][j][k]);
                endrule

                // Update flow eligibility for each flow via current neighbor,
                // based on tokens parsed in receive path.
                // TODO: In order to minimize conflict between choose buffer rules
                // and Rx processing, we could update tkn_count in Rx, but have this 
                // rule to update curr_time_in. In this case, regardless of number of
                // tokens, we only add 1 cycle overhead to Shoal processing. 
                rule update_tokens;
                    let tkn_bkt_idx <- toGet(update_tokens_fifo[i][j][k]).get;
                    $display("[SCHED %d] Increasing tokens for bkt %d to %d at time=%d", host_index[i], 
                             tkn_bkt_idx, tkn_count[i][j][k][tkn_bkt_idx] + 1, current_time);
                    tkn_count[i][j][k][tkn_bkt_idx] <= tkn_count[i][j][k][tkn_bkt_idx] + 1;
                endrule
            end
        end
    end

/*------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------*/

                                /* Interface */

/*------------------------------------------------------------------------------*/

    // The WAIT_FOR_START_SIG option sets the start flag only on getting a signal from the mac.
    // This happens when the Mac receives its first cell.
    // TODO: our code might not work with this option, because if start_flag is set on first receive, 
    // the receive path is then activated activated. But we need to ensure that coordinates
    // and schedule table have been set. May need to add checks for this.
    `ifdef WAIT_FOR_START_SIG
    for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        rule start_scheduler;
            let d <- mac.start_scheduler[i].get;
            start_flag[i] <= 1;
        endrule
    end
    `endif

    Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp1;
    Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp4;
    Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp7;
    Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(72))) temp8;
    Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp9;
    // Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp2;
    // Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp3;
    // Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp5;
    // Vector#(NUM_OF_ALTERA_PORTS, Get#(Bit#(64))) temp6;

    for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    begin
        temp1[i] = toGet(time_slots_fifo[i]);
        temp4[i] = toGet(received_host_pkt_fifo[i]);
        temp7[i] = toGet(received_wrong_dst_pkt_fifo[i]);
        temp8[i] = toGet(latency_fifo[i]);
        temp9[i] = toGet(buffer_length_fifo[i]);
    //     temp2[i] = toGet(sent_host_pkt_fifo[i]);
    //     temp3[i] = toGet(sent_fwd_pkt_fifo[i]);
    //     temp5[i] = toGet(received_fwd_pkt_fifo[i]);
    //     temp6[i] = toGet(received_corrupted_pkt_fifo[i]);
    end

    // Should prob change name from first_host_index to fpga_idx or altera_idx
    method Action start(ServerIndex first_host_index, Bit#(8) t);
        timeslot_len <= t;
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
        begin
            host_index[i] <= (first_host_index
                * fromInteger(valueof(NUM_OF_ALTERA_PORTS))) + fromInteger(i);
            `ifndef WAIT_FOR_START_SIG
            start_flag[i] <= 1;
            `endif
        end
    endmethod

    method Action stop();
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
        begin
            start_flag[i] <= 0;
            running[i] <= 0;
        end
    endmethod

    // Stat Methods
    method Action timeSlotsCount();
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
            time_slots_fifo[i].enq(num_of_time_slots_used_reg[i]);
    endmethod

	method Action receivedHostPktCount();
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
            received_host_pkt_fifo[i].enq(num_of_host_pkt_received_reg[i]);
	endmethod

	method Action receivedWrongDstPktCount();
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
            received_wrong_dst_pkt_fifo[i].enq
                (num_of_wrong_dst_pkt_received_reg[i]);
	endmethod

	method Action latency();
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
            latency_fifo[i].enq(latency_reg[i]);
	endmethod

    method Action max_fwd_buffer_length();
        for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
            buffer_length_fifo[i].enq(extend(max_fwd_buffer_length_reg[i]));
    endmethod


	// method Action sentHostPktCount();
    //     for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    //         sent_host_pkt_fifo[i].enq(host_pkt_transmitted_reg[i]);
	// endmethod

	// method Action sentFwdPktCount();
    //     for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    //         sent_fwd_pkt_fifo[i].enq(non_host_pkt_transmitted_reg[i]);
	// endmethod

    // method Action receivedFwdPktCount();
    //     for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    //         received_fwd_pkt_fifo[i].enq(num_of_fwd_pkt_received_reg[i]);
	// endmethod

	// method Action receivedCorruptedPktCount();
    //     for (Integer i = 0; i < valueof(NUM_OF_ALTERA_PORTS); i = i + 1)
    //         received_corrupted_pkt_fifo[i].enq
    //             (num_of_corrupted_pkt_received_reg[i]);
	// endmethod

	interface Get time_slots_res = temp1;
	interface Get received_host_pkt_res = temp4;
	interface Get received_wrong_dst_pkt_res = temp7;
	interface Get latency_res = temp8;
    interface Get max_fwd_buffer_length_res = temp9;
    // interface Get sent_host_pkt_res = temp2;
	// interface Get sent_fwd_pkt_res = temp3;
	// interface Get received_fwd_pkt_res = temp5;
	// interface Get received_corrupted_pkt_res = temp6;
/*------------------------------------------------------------------------------*/

endmodule
