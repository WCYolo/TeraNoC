// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// ============================================================================
// v4m_tracer_allin.svh - Vis4Mesh CSV tracer (ALL-IN-ONE, header-only)
// Include this *inside* your testbench module.
//
// It:
//  - declares tap arrays
//  - COLLECTS data via your XMR loop (embedded below)
//  - snapshots one cycle later
//  - maintains cycle to slice
//  - writes one CSV row per active directed link per cycle
//
// MsgTypes (fixed 4):
//   0: *mem.DataReadyRsp
//   1: *mem.ReadReq
//   2: *mem.WriteDoneRsp
//   3: *mem.WriteReq
//
// Transfer types:
//   0: TX, 1: Relay, 2: RX, 3: Peripheral
// ============================================================================

`ifndef V4M_TRACER_ALLIN_SVH
`define V4M_TRACER_ALLIN_SVH

// ---------- Bind to your TB signals (override with `define before include) ---
`ifndef V4M_CLK
`define V4M_CLK           clk
`endif
`ifndef V4M_RSTN
`define V4M_RSTN          rst_n
`endif
`ifndef V4M_ENABLE
`define V4M_ENABLE        1'b1
`endif

// ---------- Output path & slice size (override if you want) ------------------
`ifndef V4M_OUT_DIR
`define V4M_OUT_DIR       "v4m_out"
`endif
`ifndef V4M_OUT_FILE
`define V4M_OUT_FILE      "trace_events.csv"
`endif
`ifndef V4M_SLICE_CYCLES
`define V4M_SLICE_CYCLES  50
`endif

// ---------- Geometry & composition (defaults; override in TB if needed) -----
`ifndef W
localparam int W = NumX;
`endif
`ifndef H
localparam int H = NumY;
`endif

// ---------- Encodings --------------------------------------------------------
localparam int MT_DATAREADY_RSP = 0;
localparam int MT_READ_REQ      = 1;
localparam int MT_WRITEDONE_RSP = 2;
localparam int MT_WRITE_REQ     = 3;

localparam int TT_MESH_TX       = 0;
localparam int TT_MESH_RELAY    = 1;
localparam int TT_MESH_RX       = 2;
localparam int TT_PERIPHERAL    = 3;

// ---------- Derived sizes ----------------------------------------------------
localparam int NumRemotePortsPerTile = (NumRemoteReqPortsPerTile  - 1)
                                     + (NumRemoteRespPortsPerTile - 1);
localparam int NumWideRemoteRespPortsPerTile = NumRemoteRespPortsPerTile - 1;
localparam int NumRouterPerGroup     = NumRemotePortsPerTile * NumTilesPerGroup;
localparam int PORTS_PER_ROUTER      = 4; // N/E/S/W
localparam int TAPS_PER_GROUP        = NumRouterPerGroup * PORTS_PER_ROUTER;
localparam int GROUPS                = W * H;
localparam int N_TAPS                = GROUPS * TAPS_PER_GROUP;

function automatic int unsigned v4m_edge_dst_id(input int unsigned x, input int unsigned y,
                                                 input int unsigned port);
  int unsigned dst_x;
  int unsigned dst_y;
  begin
    dst_x = x;
    dst_y = y;
    case (port)
      0: begin // North
        if (NocTopology == 1) dst_y = (y + 1) % H;
        else if (y + 1 < H)  dst_y = y + 1;
        else                 return GROUPS;
      end
      1: begin // East
        if (NocTopology == 1) dst_x = (x + 1) % W;
        else if (x + 1 < W)  dst_x = x + 1;
        else                 return GROUPS;
      end
      2: begin // South
        if (NocTopology == 1) dst_y = (y == 0) ? H - 1 : y - 1;
        else if (y > 0)      dst_y = y - 1;
        else                 return GROUPS;
      end
      3: begin // West
        if (NocTopology == 1) dst_x = (x == 0) ? W - 1 : x - 1;
        else if (x > 0)      dst_x = x - 1;
        else                 return GROUPS;
      end
      default: return GROUPS;
    endcase
    return dst_y * W + dst_x;
  end
endfunction

// ---------- Tap arrays (written below by the data-collection block) ----------
logic                  tap_valid   [N_TAPS];
logic            [1:0] tap_msg_idx [N_TAPS];    // 0..3
logic            [1:0] tap_tt      [N_TAPS];    // 0..3
int unsigned           tap_channel [N_TAPS];    // physical NoC id
int unsigned           edge_src_id [N_TAPS];    // current hop src (group id = y*W+x)
int unsigned           edge_dst_id [N_TAPS];    // current hop dst (group id)
int unsigned           tap_src_id  [N_TAPS];    // packet NI src (group id)
int unsigned           tap_dst_id  [N_TAPS];    // packet NI dst (group id)

// ============================================================================
// DATA COLLECTION  —  your XMR hook embedded here (one writer total)
// NOTE: The hierarchical paths below use your example (dut...).
// If your TB name/hierarchy differs when you `include` this file, either:
//   - change the base path below, or
//   - `define V4M_BASE` to your path (e.g. `"tb_top.dut"`) and replace uses.
//
// Port-direction mapping:
//   0:N(y+1), 1:E(x+1), 2:S(y-1), 3:W(x-1)
// Mesh border neighbors are skipped; torus border neighbors wrap.
// ============================================================================
genvar group_x, group_y;
genvar router_id, router_port_id;

generate
  for (group_y = 0; group_y < H; group_y++) begin
    for (group_x = 0; group_x < W; group_x++) begin
      localparam int base_id = group_y * W + group_x;

      for (router_id = 0; router_id < NumRouterPerGroup; router_id++) begin

        if (router_id < NumNarrowRemoteReqPortsPerTile * NumTilesPerGroup) begin
          // ------------------- narrow req ports -------------------
          for (router_port_id = 0; router_port_id < 4; router_port_id++) begin
            localparam int link_id = base_id * NumRouterPerGroup * 4 + router_id * 4 + router_port_id;
            always_ff @(posedge `V4M_CLK) begin
              tap_msg_idx [link_id] <= MT_READ_REQ;     // req (as given)
              tap_tt      [link_id] <= TT_MESH_RELAY;   // relay for now
              tap_channel [link_id] <= router_id;       // channel index policy is yours

              edge_src_id [link_id] <= base_id;
              edge_dst_id [link_id] <= v4m_edge_dst_id(group_x, group_y, router_port_id);

              tap_src_id  [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_narrow_req_out_trans
                                        [router_id/NumNarrowRemoteReqPortsPerTile]
                                        [router_id%NumNarrowRemoteReqPortsPerTile]
                                        [router_port_id].hdr.src_id.y * W
                                    + dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_narrow_req_out_trans
                                        [router_id/NumNarrowRemoteReqPortsPerTile]
                                        [router_id%NumNarrowRemoteReqPortsPerTile]
                                        [router_port_id].hdr.src_id.x;

              tap_dst_id  [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_narrow_req_out_trans
                                        [router_id/NumNarrowRemoteReqPortsPerTile]
                                        [router_id%NumNarrowRemoteReqPortsPerTile]
                                        [router_port_id].hdr.dst_id.y * W
                                    + dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_narrow_req_out_trans
                                        [router_id/NumNarrowRemoteReqPortsPerTile]
                                        [router_id%NumNarrowRemoteReqPortsPerTile]
                                        [router_port_id].hdr.dst_id.x;

              tap_valid   [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_narrow_req_valid_out_trans
                                        [router_id/NumNarrowRemoteReqPortsPerTile]
                                        [router_id%NumNarrowRemoteReqPortsPerTile]
                                        [router_port_id]
                                    && dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_narrow_req_ready_in_trans
                                        [router_id/NumNarrowRemoteReqPortsPerTile]
                                        [router_id%NumNarrowRemoteReqPortsPerTile]
                                        [router_port_id];
            end
          end
        end else if (router_id < (NumNarrowRemoteReqPortsPerTile + NumWideRemoteReqPortsPerTile) * NumTilesPerGroup) begin
          // ------------------- wide req ports ---------------------
          localparam int offset_router_id = router_id - NumNarrowRemoteReqPortsPerTile * NumTilesPerGroup;
          for (router_port_id = 0; router_port_id < 4; router_port_id++) begin
            localparam int link_id = base_id * NumRouterPerGroup * 4 + router_id * 4 + router_port_id;
            always_ff @(posedge `V4M_CLK) begin
              tap_msg_idx [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_out_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id].payload.wen
                                        ? MT_WRITE_REQ : MT_READ_REQ;

              tap_tt      [link_id] <= TT_MESH_RELAY;
              tap_channel [link_id] <= router_id;

              edge_src_id [link_id] <= base_id;
              edge_dst_id [link_id] <= v4m_edge_dst_id(group_x, group_y, router_port_id);

              tap_src_id  [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_out_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id].hdr.src_id.y * W
                                    + dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_out_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id].hdr.src_id.x;

              tap_dst_id  [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_out_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id].hdr.dst_id.y * W
                                    + dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_out_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id].hdr.dst_id.x;

              tap_valid   [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_valid_out_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id]
                                    && dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_wide_req_ready_in_trans
                                        [offset_router_id/NumWideRemoteReqPortsPerTile]
                                        [offset_router_id%NumWideRemoteReqPortsPerTile]
                                        [router_port_id];
            end
          end

        end else begin
          // ------------------- wide resp ports --------------------
          localparam int offset_router_id = router_id - (NumNarrowRemoteReqPortsPerTile + NumWideRemoteReqPortsPerTile) * NumTilesPerGroup;
          for (router_port_id = 0; router_port_id < 4; router_port_id++) begin
            localparam int link_id = base_id * NumRouterPerGroup * 4 + router_id * 4 + router_port_id;
            always_ff @(posedge `V4M_CLK) begin
              tap_msg_idx [link_id] <= MT_DATAREADY_RSP;  // resp (as given)
              tap_tt      [link_id] <= TT_MESH_RELAY;
              tap_channel [link_id] <= router_id;

              edge_src_id [link_id] <= base_id;
              edge_dst_id [link_id] <= v4m_edge_dst_id(group_x, group_y, router_port_id);

              tap_src_id  [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_resp_out_trans
                                        [offset_router_id/NumWideRemoteRespPortsPerTile]
                                        [offset_router_id%NumWideRemoteRespPortsPerTile+1]
                                        [router_port_id].hdr.src_id.y * W
                                    + dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_resp_out_trans
                                        [offset_router_id/NumWideRemoteRespPortsPerTile]
                                        [offset_router_id%NumWideRemoteRespPortsPerTile+1]
                                        [router_port_id].hdr.src_id.x;

              tap_dst_id  [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_resp_out_trans
                                        [offset_router_id/NumWideRemoteRespPortsPerTile]
                                        [offset_router_id%NumWideRemoteRespPortsPerTile+1]
                                        [router_port_id].hdr.dst_id.y * W
                                    + dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_resp_out_trans
                                        [offset_router_id/NumWideRemoteRespPortsPerTile]
                                        [offset_router_id%NumWideRemoteRespPortsPerTile+1]
                                        [router_port_id].hdr.dst_id.x;

              tap_valid   [link_id] <= dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_resp_valid_out_trans
                                        [offset_router_id/NumWideRemoteRespPortsPerTile]
                                        [offset_router_id%NumWideRemoteRespPortsPerTile+1]
                                        [router_port_id]
                                    && dut.i_mempool_cluster.gen_groups_x[group_x].gen_groups_y[group_y].gen_rtl_group.i_group
                                      .floo_tcdm_resp_ready_in_trans
                                        [offset_router_id/NumWideRemoteRespPortsPerTile]
                                        [offset_router_id%NumWideRemoteRespPortsPerTile+1]
                                        [router_port_id];
            end
          end
        end
      end
    end
  end
endgenerate

// ---------- Stage-1 snapshot for logging -------------------------------------
logic                  v_q   [N_TAPS];
logic            [1:0] mt_q  [N_TAPS];
logic            [1:0] tt_q  [N_TAPS];
int unsigned           ch_q  [N_TAPS];
int unsigned           es_q  [N_TAPS];
int unsigned           ed_q  [N_TAPS];
int unsigned           ps_q  [N_TAPS];
int unsigned           pd_q  [N_TAPS];

int i_v4m_reg;
always_ff @(posedge `V4M_CLK or negedge `V4M_RSTN) begin
  if (!`V4M_RSTN) begin
    for (i_v4m_reg = 0; i_v4m_reg < N_TAPS; i_v4m_reg++) begin
      v_q[i_v4m_reg]  <= 1'b0;
      mt_q[i_v4m_reg] <= '0;
      tt_q[i_v4m_reg] <= '0;
      ch_q[i_v4m_reg] <= '0;
      es_q[i_v4m_reg] <= '0;
      ed_q[i_v4m_reg] <= '0;
      ps_q[i_v4m_reg] <= '0;
      pd_q[i_v4m_reg] <= '0;
    end
  end else if (`V4M_ENABLE) begin
    for (i_v4m_reg = 0; i_v4m_reg < N_TAPS; i_v4m_reg++) begin
      v_q[i_v4m_reg]  <= tap_valid[i_v4m_reg];
      mt_q[i_v4m_reg] <= tap_msg_idx[i_v4m_reg];
      tt_q[i_v4m_reg] <= tap_tt[i_v4m_reg];
      ch_q[i_v4m_reg] <= tap_channel[i_v4m_reg];
      es_q[i_v4m_reg] <= edge_src_id[i_v4m_reg];
      ed_q[i_v4m_reg] <= edge_dst_id[i_v4m_reg];
      ps_q[i_v4m_reg] <= tap_src_id[i_v4m_reg];
      pd_q[i_v4m_reg] <= tap_dst_id[i_v4m_reg];
    end
  end
end

// ---------- Time & file I/O ---------------------------------------------------
int unsigned V4M_cycle;
int unsigned V4M_slice;
integer      V4M_fd;

always_ff @(posedge `V4M_CLK or negedge `V4M_RSTN) begin
  if (!`V4M_RSTN) begin
    V4M_cycle <= 0;
    V4M_slice <= 0;
  end else if (`V4M_ENABLE) begin
    V4M_cycle <= V4M_cycle + 1;
    if (`V4M_SLICE_CYCLES == 0) V4M_slice <= 0;
    else                        V4M_slice <= V4M_cycle / `V4M_SLICE_CYCLES;
  end
end

initial begin
  void'($system($sformatf("mkdir -p %s", `V4M_OUT_DIR)));
  V4M_fd = $fopen({`V4M_OUT_DIR,"/",`V4M_OUT_FILE}, "w");
  if (!V4M_fd) $fatal(1, "V4M: cannot open %s/%s", `V4M_OUT_DIR, `V4M_OUT_FILE);
  $fwrite(V4M_fd, "slice,edge_src,edge_dst,tt,mt,ch,flits,pkt_src,pkt_dst\n");
end

// ---------- Logger: one row per active tap per cycle -------------------------
int j_v4m;
always_ff @(posedge `V4M_CLK) if (`V4M_ENABLE) begin
  for (j_v4m = 0; j_v4m < N_TAPS; j_v4m++) begin
    if (v_q[j_v4m]) begin
      if (es_q[j_v4m] < (W*H) && ed_q[j_v4m] < (W*H)) begin
        $fwrite(V4M_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
          V4M_slice, es_q[j_v4m], ed_q[j_v4m], tt_q[j_v4m], mt_q[j_v4m], ch_q[j_v4m],
          1, ps_q[j_v4m], pd_q[j_v4m]); // flits=1 per handshake
      end
    end
  end
end

final begin
  if (V4M_fd) $fclose(V4M_fd);
  $display("V4M: wrote %s/%s", `V4M_OUT_DIR, `V4M_OUT_FILE);
end

`endif // V4M_TRACER_ALLIN_SVH
