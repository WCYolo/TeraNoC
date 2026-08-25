// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Yichao Zhang <yiczhang@iis.ee.ethz.ch>

`include "common_cells/registers.svh"

module terapool_cluster_floonoc_wrapper
  import mempool_pkg::*;
  import cf_math_pkg::idx_width;
  import floo_pkg::*;
  import floo_terapool_noc_pkg::*;
#(
  // TCDM
  parameter addr_t        TCDMBaseAddr   = 32'b0,
  // Boot address
  parameter logic [31:0]  BootAddr       = 32'h0000_0000,
  // Dependant parameters. DO NOT CHANGE!
  parameter int unsigned  NumDMAReq      = NumGroups * NumDmasPerGroup,
  parameter int unsigned  NumAXIMasters  = NumGroups * NumAXIMastersPerGroup
) (
  // Clock and reset
  input  logic                          clk_i,
  input  logic                          rst_ni,
  input  logic                          testmode_i,

  // Scan chain
  input  logic                          scan_enable_i,
  input  logic                          scan_data_i,
  output logic                          scan_data_o,

  // Wake-up signal
  input  logic         [NumCores-1:0]   wake_up_i,

  // RO-Cache configuration
  input  ro_cache_ctrl_t                ro_cache_ctrl_i,

  // DMA request
  input  dma_req_t     [NumGroups-1:0]  dma_req_i,
  input  logic         [NumGroups-1:0]  dma_req_valid_i,
  output logic         [NumGroups-1:0]  dma_req_ready_o,

  // DMA status
  output dma_meta_t    [NumGroups-1:0]  dma_meta_o,

  // AXI Interface
  input  floo_req_t    [NumAXIMasters-1:0] floo_axi_req_i,
  input  floo_rsp_t    [NumAXIMasters-1:0] floo_axi_rsp_i,
  input  floo_wide_t   [NumAXIMasters-1:0] floo_axi_wide_i,
  output floo_req_t    [NumAXIMasters-1:0] floo_axi_req_o,
  output floo_rsp_t    [NumAXIMasters-1:0] floo_axi_rsp_o,
  output floo_wide_t   [NumAXIMasters-1:0] floo_axi_wide_o
);

  /*********************
   *  Control Signals  *
   *********************/
  logic [NumCores-1:0] wake_up_q;
  `FF(wake_up_q, wake_up_i, '0, clk_i, rst_ni);

  ro_cache_ctrl_t [NumGroups-1:0] ro_cache_ctrl_q;
  for (genvar g = 0; unsigned'(g) < NumGroups; g++) begin : gen_ro_cache_ctrl_q
    `FF(ro_cache_ctrl_q[g], ro_cache_ctrl_i, ro_cache_ctrl_default, clk_i, rst_ni);
  end : gen_ro_cache_ctrl_q

  /*********
   *  DMA  *
   *********/
  dma_req_t  [NumGroups-1:0] dma_req_group, dma_req_group_q;
  logic      [NumGroups-1:0] dma_req_group_valid, dma_req_group_q_valid;
  logic      [NumGroups-1:0] dma_req_group_ready, dma_req_group_q_ready;
  dma_meta_t [NumGroups-1:0] dma_meta, dma_meta_q;

  assign dma_req_group = dma_req_i;
  assign dma_req_group_valid = dma_req_valid_i;
  assign dma_req_ready_o = dma_req_group_ready;
  assign dma_meta_o = dma_meta_q;

  `FF(dma_meta_q, dma_meta, '0, clk_i, rst_ni);

  for (genvar g = 0; unsigned'(g) < NumGroups; g++) begin: gen_dma_req_group_register
    spill_register #(
      .T(dma_req_t)
    ) i_dma_req_group_register (
      .clk_i  (clk_i                   ),
      .rst_ni (rst_ni                  ),
      .data_i (dma_req_group[g]        ),
      .valid_i(dma_req_group_valid[g]  ),
      .ready_o(dma_req_group_ready[g]  ),
      .data_o (dma_req_group_q[g]      ),
      .valid_o(dma_req_group_q_valid[g]),
      .ready_i(dma_req_group_q_ready[g])
    );
  end : gen_dma_req_group_register

  /************
   *  Groups  *
   ************/
  // FlooNoC TCDM interfaces
  floo_tcdm_req_if_t [NumX-1:0][NumY-1:0][West:North] floo_tcdm_req_out, floo_tcdm_req_in;
  floo_tcdm_rsp_if_t [NumX-1:0][NumY-1:0][West:North] floo_tcdm_rsp_out, floo_tcdm_rsp_in;

  // X-axis TCDM NoC feedthrough interfaces
  floo_tcdm_req_if_t [NumX-1:0][NumY-1:0][NumTcdmFtDirections-1:0] ft_tcdm_req_in, ft_tcdm_req_out;
  floo_tcdm_rsp_if_t [NumX-1:0][NumY-1:0][NumTcdmFtDirections-1:0] ft_tcdm_rsp_in, ft_tcdm_rsp_out;

  // FlooNoC AXI interfaces
  floo_terapool_noc_pkg::floo_req_t  [NumX-1:0][NumY-1:0][West:North] floo_axi_req_out,  floo_axi_req_in;
  floo_terapool_noc_pkg::floo_rsp_t  [NumX-1:0][NumY-1:0][West:North] floo_axi_rsp_out,  floo_axi_rsp_in;
  floo_terapool_noc_pkg::floo_wide_t [NumX-1:0][NumY-1:0][West:North] floo_axi_wide_out, floo_axi_wide_in;

  // Chimney configuration
  localparam floo_pkg::chimney_cfg_t ChimneyCfgN = floo_pkg::set_ports(floo_pkg::ChimneyDefaultCfg, 1'b0, 1'b0);
  localparam floo_pkg::chimney_cfg_t ChimneyCfgW = floo_pkg::set_ports(floo_pkg::ChimneyDefaultCfg, 1'b1, 1'b0);

  // X-shuffled physical group[x][y] -> logical group id.
  // y=3:  7   3   11  15
  // y=2:  6   2   10  14
  // y=1:  5   1    9  13
  // y=0:  4   0    8  12
  //       x=0 x=1 x=2 x=3
  // The logical group ID is computed from this mapping in gen_groups_x/y.

  for (genvar x = 0; x < NumX; x++) begin : gen_groups_x
    for (genvar y = 0; y < NumY; y++) begin : gen_groups_y
      localparam int unsigned MeshGroupId     = x * NumY + y;
      localparam int unsigned ShuffledGroupX  = (x == 0) ? 1 : ((x == 1) ? 0 : x);
      localparam int unsigned ShuffledGroupId = ShuffledGroupX * NumY + y;
      localparam int unsigned TcdmGroupId     = (NocTopology == 1) ? ShuffledGroupId : MeshGroupId;

      localparam int unsigned TcdmGroupX = TcdmGroupId / NumY;
      localparam int unsigned TcdmGroupY = TcdmGroupId % NumY;

      group_xy_id_t group_id;
      assign group_id = '{x:TcdmGroupX, y:TcdmGroupY, port_id:1'b0};

      localparam tcdm_axis_mode_e TcdmEwAdapterMode = (NocTopology == 1) ? (
        (x == 0) ? TCDM_AXIS_SIDE0_EDGE_BRIDGE :
        (x == 1) ? TCDM_AXIS_SWAP_DATA_PASS_FT :
        (x == 2) ? TCDM_AXIS_DEFAULT : TCDM_AXIS_SIDE1_EDGE_BRIDGE
      ) : TCDM_AXIS_DEFAULT;

      // TCDM-only x-shuffled torus wiring. Long physical links, including the
      // logical x wrap, use the pipelined feedthrough path inside each group.
      if (NocTopology == 1) begin : gen_tcdm_x_shuffled_torus
        if (x == 0) begin : gen_shuffled_tcdm_rows
          assign floo_tcdm_req_in[0][y][West] = '0;
          assign floo_tcdm_rsp_in[0][y][West] = '0;
          assign ft_tcdm_req_in[0][y][TcdmFtWest] = '0;
          assign ft_tcdm_rsp_in[0][y][TcdmFtWest] = '0;

          assign floo_tcdm_req_in[3][y][East] = '0;
          assign floo_tcdm_rsp_in[3][y][East] = '0;
          assign ft_tcdm_req_in[3][y][TcdmFtEast] = '0;
          assign ft_tcdm_rsp_in[3][y][TcdmFtEast] = '0;

          // Request links in both directions.
          assign ft_tcdm_req_in[0][y][TcdmFtEast] = floo_tcdm_req_out[1][y][West];

          assign ft_tcdm_req_in[1][y][TcdmFtWest] = floo_tcdm_req_out[0][y][East];
          assign floo_tcdm_req_in[2][y][West] = ft_tcdm_req_out[1][y][TcdmFtEast];

          assign floo_tcdm_req_in[3][y][West] = floo_tcdm_req_out[2][y][East];

          assign ft_tcdm_req_in[2][y][TcdmFtEast] = ft_tcdm_req_out[3][y][TcdmFtWest];
          assign floo_tcdm_req_in[1][y][East] = ft_tcdm_req_out[2][y][TcdmFtWest];

          assign floo_tcdm_req_in[1][y][West] = ft_tcdm_req_out[0][y][TcdmFtEast];

          assign ft_tcdm_req_in[1][y][TcdmFtEast] = floo_tcdm_req_out[2][y][West];
          assign floo_tcdm_req_in[0][y][East] = ft_tcdm_req_out[1][y][TcdmFtWest];

          assign floo_tcdm_req_in[2][y][East] = floo_tcdm_req_out[3][y][West];

          assign ft_tcdm_req_in[2][y][TcdmFtWest] = floo_tcdm_req_out[1][y][East];
          assign ft_tcdm_req_in[3][y][TcdmFtWest] = ft_tcdm_req_out[2][y][TcdmFtEast];

          // Response links mirror the request links.
          assign ft_tcdm_rsp_in[0][y][TcdmFtEast] = floo_tcdm_rsp_out[1][y][West];

          assign ft_tcdm_rsp_in[1][y][TcdmFtWest] = floo_tcdm_rsp_out[0][y][East];
          assign floo_tcdm_rsp_in[2][y][West] = ft_tcdm_rsp_out[1][y][TcdmFtEast];

          assign floo_tcdm_rsp_in[3][y][West] = floo_tcdm_rsp_out[2][y][East];

          assign ft_tcdm_rsp_in[2][y][TcdmFtEast] = ft_tcdm_rsp_out[3][y][TcdmFtWest];
          assign floo_tcdm_rsp_in[1][y][East] = ft_tcdm_rsp_out[2][y][TcdmFtWest];

          assign floo_tcdm_rsp_in[1][y][West] = ft_tcdm_rsp_out[0][y][TcdmFtEast];

          assign ft_tcdm_rsp_in[1][y][TcdmFtEast] = floo_tcdm_rsp_out[2][y][West];
          assign floo_tcdm_rsp_in[0][y][East] = ft_tcdm_rsp_out[1][y][TcdmFtWest];

          assign floo_tcdm_rsp_in[2][y][East] = floo_tcdm_rsp_out[3][y][West];

          assign ft_tcdm_rsp_in[2][y][TcdmFtWest] = floo_tcdm_rsp_out[1][y][East];
          assign ft_tcdm_rsp_in[3][y][TcdmFtWest] = ft_tcdm_rsp_out[2][y][TcdmFtEast];
        end
      end else begin : gen_tcdm_mesh
        assign ft_tcdm_req_in[x][y] = '0;
        assign ft_tcdm_rsp_in[x][y] = '0;

        if (x == 0) begin
          assign floo_tcdm_req_in[x][y][West] = '0;
          assign floo_tcdm_rsp_in[x][y][West] = '0;
          assign floo_tcdm_req_in[x][y][East] = floo_tcdm_req_out[x+1][y][West];
          assign floo_tcdm_rsp_in[x][y][East] = floo_tcdm_rsp_out[x+1][y][West];
        end else if (x == NumX-1) begin
          assign floo_tcdm_req_in[x][y][East] = '0;
          assign floo_tcdm_rsp_in[x][y][East] = '0;
          assign floo_tcdm_req_in[x][y][West] = floo_tcdm_req_out[x-1][y][East];
          assign floo_tcdm_rsp_in[x][y][West] = floo_tcdm_rsp_out[x-1][y][East];
        end else begin
          assign floo_tcdm_req_in[x][y][East] = floo_tcdm_req_out[x+1][y][West];
          assign floo_tcdm_rsp_in[x][y][East] = floo_tcdm_rsp_out[x+1][y][West];
          assign floo_tcdm_req_in[x][y][West] = floo_tcdm_req_out[x-1][y][East];
          assign floo_tcdm_rsp_in[x][y][West] = floo_tcdm_rsp_out[x-1][y][East];
        end
      end

      // The y axis is not shuffled. Keep every y link direct, including the
      // torus wrap between y=0 and y=NumY-1. No y feedthrough path is used.
      if (y == 0) begin : gen_tcdm_y_south
        if (NocTopology == 1) begin : gen_torus_wrap
          assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][NumY-1][North];
          assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][NumY-1][North];
        end else begin : gen_no_wrap
          assign floo_tcdm_req_in[x][y][South] = '0;
          assign floo_tcdm_rsp_in[x][y][South] = '0;
        end

        assign floo_tcdm_req_in[x][y][North] = floo_tcdm_req_out[x][y+1][South];
        assign floo_tcdm_rsp_in[x][y][North] = floo_tcdm_rsp_out[x][y+1][South];
      end else if (y == NumY-1) begin : gen_tcdm_y_north
        if (NocTopology == 1) begin : gen_torus_wrap
          assign floo_tcdm_req_in[x][y][North] = floo_tcdm_req_out[x][0][South];
          assign floo_tcdm_rsp_in[x][y][North] = floo_tcdm_rsp_out[x][0][South];
        end else begin : gen_no_wrap
          assign floo_tcdm_req_in[x][y][North] = '0;
          assign floo_tcdm_rsp_in[x][y][North] = '0;
        end

        assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][y-1][North];
        assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][y-1][North];
      end else begin : gen_tcdm_y_internal
        assign floo_tcdm_req_in[x][y][North] = floo_tcdm_req_out[x][y+1][South];
        assign floo_tcdm_rsp_in[x][y][North] = floo_tcdm_rsp_out[x][y+1][South];
        assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][y-1][North];
        assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][y-1][North];
      end

      // AXI remains a physical mesh. Only logical group endpoints and external
      // HBM channel IDs move to match the x-shuffled floorplan.
      if (x == 0) begin : gen_hbm_chimney_west

        // AXI East
        assign floo_axi_req_in[x][y][East]  = floo_axi_req_out[x+1][y][West];
        assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_out[x+1][y][West];
        assign floo_axi_wide_in[x][y][East] = floo_axi_wide_out[x+1][y][West];

        localparam int unsigned HbmWestId = (NocTopology == 1) ? 4 + y : y;

        // AXI West
        assign floo_axi_req_in[x][y][West]  = floo_axi_req_i[HbmWestId];
        assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_i[HbmWestId];
        assign floo_axi_wide_in[x][y][West] = floo_axi_wide_i[HbmWestId];
        assign floo_axi_wide_o[HbmWestId]   = floo_axi_wide_out[x][y][West];
        assign floo_axi_req_o[HbmWestId]    = floo_axi_req_out[x][y][West];
        assign floo_axi_rsp_o[HbmWestId]    = floo_axi_rsp_out[x][y][West];

      end else if (x == NumX-1) begin : gen_hbm_chimney_east
        // AXI West
        assign floo_axi_req_in[x][y][West]  = floo_axi_req_out[x-1][y][East];
        assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_out[x-1][y][East];
        assign floo_axi_wide_in[x][y][West] = floo_axi_wide_out[x-1][y][East];

        localparam int unsigned HbmEastId = 12 + y;

        // AXI East
        assign floo_axi_req_in[x][y][East]  = floo_axi_req_i[HbmEastId];
        assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_i[HbmEastId];
        assign floo_axi_wide_in[x][y][East] = floo_axi_wide_i[HbmEastId];
        assign floo_axi_wide_o[HbmEastId]   = floo_axi_wide_out[x][y][East];
        assign floo_axi_req_o[HbmEastId]    = floo_axi_req_out[x][y][East];
        assign floo_axi_rsp_o[HbmEastId]    = floo_axi_rsp_out[x][y][East];

      end else begin : gen_hor_connections
        // East
        assign floo_axi_req_in[x][y][East]  = floo_axi_req_out[x+1][y][West];
        assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_out[x+1][y][West];
        assign floo_axi_wide_in[x][y][East] = floo_axi_wide_out[x+1][y][West];

        // West
        assign floo_axi_req_in[x][y][West]  = floo_axi_req_out[x-1][y][East];
        assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_out[x-1][y][East];
        assign floo_axi_wide_in[x][y][West] = floo_axi_wide_out[x-1][y][East];
      end

      if (y == 0) begin : gen_hbm_chimney_south
        // AXI North
        assign floo_axi_req_in  [x][y][North] = floo_axi_req_out  [x][y+1][South];
        assign floo_axi_rsp_in  [x][y][North] = floo_axi_rsp_out  [x][y+1][South];
        assign floo_axi_wide_in [x][y][North] = floo_axi_wide_out [x][y+1][South];

        localparam int unsigned HbmSouthId = (NocTopology == 1) ?
            ((x < 2) ? 1 - x : x + 6) :
            ((x < 2) ? 5 - x : x + 6);

        // AXI South
        assign floo_axi_req_in[x][y][South]  = floo_axi_req_i[HbmSouthId];
        assign floo_axi_rsp_in[x][y][South]  = floo_axi_rsp_i[HbmSouthId];
        assign floo_axi_wide_in[x][y][South] = floo_axi_wide_i[HbmSouthId];
        assign floo_axi_wide_o[HbmSouthId]   = floo_axi_wide_out[x][y][South];
        assign floo_axi_req_o[HbmSouthId]    = floo_axi_req_out[x][y][South];
        assign floo_axi_rsp_o[HbmSouthId]    = floo_axi_rsp_out[x][y][South];

      end else if (y == NumY-1) begin
        // AXI South
        assign floo_axi_req_in [x][y][South] = floo_axi_req_out [x][y-1][North];
        assign floo_axi_rsp_in [x][y][South] = floo_axi_rsp_out [x][y-1][North];
        assign floo_axi_wide_in[x][y][South] = floo_axi_wide_out[x][y-1][North];

        localparam int unsigned HbmNorthId = (NocTopology == 1) ?
            ((x < 2) ? x + 2 : 13 - x) :
            ((x < 2) ? x + 6 : 13 - x);

        // AXI North
        assign floo_axi_req_in[x][y][North]  = floo_axi_req_i[HbmNorthId];
        assign floo_axi_rsp_in[x][y][North]  = floo_axi_rsp_i[HbmNorthId];
        assign floo_axi_wide_in[x][y][North] = floo_axi_wide_i[HbmNorthId];
        assign floo_axi_wide_o[HbmNorthId]   = floo_axi_wide_out[x][y][North];
        assign floo_axi_req_o[HbmNorthId]    = floo_axi_req_out[x][y][North];
        assign floo_axi_rsp_o[HbmNorthId]    = floo_axi_rsp_out[x][y][North];

      end else begin
        // North
        assign floo_axi_req_in  [x][y][North] = floo_axi_req_out  [x][y+1][South];
        assign floo_axi_rsp_in  [x][y][North] = floo_axi_rsp_out  [x][y+1][South];
        assign floo_axi_wide_in [x][y][North] = floo_axi_wide_out [x][y+1][South];

        // South
        assign floo_axi_req_in  [x][y][South] = floo_axi_req_out  [x][y-1][North];
        assign floo_axi_rsp_in  [x][y][South] = floo_axi_rsp_out  [x][y-1][North];
        assign floo_axi_wide_in [x][y][South] = floo_axi_wide_out [x][y-1][North];
      end

      // The existing post-layout group has no feedthrough/swap ports, so the
      // shuffled torus must instantiate the RTL wrapper at every physical slot.
      if (PostLayoutGr & (NocTopology == 0) & (x == 0) & (y == 0)) begin : gen_postly_group
        mempool_group_floonoc_wrapper_postlayout i_group (
          .clk_i              (clk_i),
          .rst_ni             (rst_ni),
          .testmode_i         (testmode_i),
          .scan_enable_i      (scan_enable_i),
          .scan_data_i        (/* Unconnected */),
          .scan_data_o        (/* Unconnected */),
          .group_id_i         (group_id_t'({group_id.x, group_id.y})),
          .floo_id_i          (id_t'(GroupX0Y0 + TcdmGroupId)),
          .route_table_i      (floo_terapool_noc_pkg::RoutingTables[GroupX0Y0 + TcdmGroupId]),
          // TCDM Router interface
          .floo_tcdm_req_o    (floo_tcdm_req_out[x][y]),
          .floo_tcdm_rsp_o    (floo_tcdm_rsp_out[x][y]),
          .floo_tcdm_req_i    (floo_tcdm_req_in[x][y]),
          .floo_tcdm_rsp_i    (floo_tcdm_rsp_in[x][y]),
          .wake_up_i          (wake_up_q[TcdmGroupId*NumCoresPerGroup +: NumCoresPerGroup]),
          .ro_cache_ctrl_i    (ro_cache_ctrl_q[TcdmGroupId]),
          // DMA request
          .dma_req_i          (dma_req_group_q[TcdmGroupId]),
          .dma_req_valid_i    (dma_req_group_q_valid[TcdmGroupId]),
          .dma_req_ready_o    (dma_req_group_q_ready[TcdmGroupId]),
          // DMA status
          .dma_meta_o_backend_idle_   (dma_meta[TcdmGroupId][1]),
          .dma_meta_o_trans_complete_ (dma_meta[TcdmGroupId][0]),
          // AXI Router interface
          .floo_axi_req_o     (floo_axi_req_out[x][y]),
          .floo_axi_rsp_o     (floo_axi_rsp_out[x][y]),
          .floo_axi_wide_o    (floo_axi_wide_out[x][y]),
          .floo_axi_req_i     (floo_axi_req_in[x][y]),
          .floo_axi_rsp_i     (floo_axi_rsp_in[x][y]),
          .floo_axi_wide_i    (floo_axi_wide_in[x][y])
        );

      end else begin : gen_rtl_group
        mempool_group_floonoc_wrapper #(
          .TCDMBaseAddr       (TCDMBaseAddr),
          .BootAddr           (BootAddr)
        ) i_group (
          .clk_i              (clk_i),
          .rst_ni             (rst_ni),
          .testmode_i         (testmode_i),
          .scan_enable_i      (scan_enable_i),
          .scan_data_i        (/* Unconnected */),
          .scan_data_o        (/* Unconnected */),
          .group_id_i         (group_id_t'({group_id.x, group_id.y})),
          .floo_id_i          (id_t'(GroupX0Y0 + TcdmGroupId)),
          .route_table_i      (floo_terapool_noc_pkg::RoutingTables[GroupX0Y0 + TcdmGroupId]),
          // TCDM Router interface
          .floo_tcdm_req_o    (floo_tcdm_req_out[x][y]),
          .floo_tcdm_rsp_o    (floo_tcdm_rsp_out[x][y]),
          .floo_tcdm_req_i    (floo_tcdm_req_in[x][y]),
          .floo_tcdm_rsp_i    (floo_tcdm_rsp_in[x][y]),
          // X-axis TCDM NoC feedthrough and physical-port adapter
          .ft_tcdm_req_i          (ft_tcdm_req_in[x][y]),
          .ft_tcdm_req_o          (ft_tcdm_req_out[x][y]),
          .ft_tcdm_rsp_i          (ft_tcdm_rsp_in[x][y]),
          .ft_tcdm_rsp_o          (ft_tcdm_rsp_out[x][y]),
          .tcdm_ew_adapter_mode_i (TcdmEwAdapterMode),
          .wake_up_i          (wake_up_q[TcdmGroupId*NumCoresPerGroup +: NumCoresPerGroup]),
          .ro_cache_ctrl_i    (ro_cache_ctrl_q[TcdmGroupId]),
          // DMA request
          .dma_req_i          (dma_req_group_q[TcdmGroupId]),
          .dma_req_valid_i    (dma_req_group_q_valid[TcdmGroupId]),
          .dma_req_ready_o    (dma_req_group_q_ready[TcdmGroupId]),
          // DMA status
          .dma_meta_o         (dma_meta[TcdmGroupId]),
          // AXI Router interface
          .floo_axi_req_o     (floo_axi_req_out[x][y]),
          .floo_axi_rsp_o     (floo_axi_rsp_out[x][y]),
          .floo_axi_wide_o    (floo_axi_wide_out[x][y]),
          .floo_axi_req_i     (floo_axi_req_in[x][y]),
          .floo_axi_rsp_i     (floo_axi_rsp_in[x][y]),
          .floo_axi_wide_i    (floo_axi_wide_in[x][y])
        );
      end
    end : gen_groups_y
  end : gen_groups_x

  /****************
   *  Assertions  *
   ****************/

  if (NumCores > 1024)
    $fatal(1, "[mempool] MemPool is currently limited to 1024 cores.");

  if (NumTiles < NumGroups)
    $fatal(1, "[mempool] MemPool requires more tiles than groups.");

  if (NumCores != NumTiles * NumCoresPerTile)
    $fatal(1, "[mempool] The number of cores is not divisible by the number of cores per tile.");

  if (BankingFactor < 1)
    $fatal(1, "[mempool] The banking factor must be a positive integer.");

  if (BankingFactor != 2**$clog2(BankingFactor))
    $fatal(1, "[mempool] The banking factor must be a power of two.");

endmodule : terapool_cluster_floonoc_wrapper
