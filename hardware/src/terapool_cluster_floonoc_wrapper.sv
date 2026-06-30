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

  // TCDM NoC feedthrough interfaces
  floo_tcdm_req_if_t [NumX-1:0][NumY-1:0][West:North] ft_tcdm_req_in, ft_tcdm_req_out;
  floo_tcdm_rsp_if_t [NumX-1:0][NumY-1:0][West:North] ft_tcdm_rsp_in, ft_tcdm_rsp_out;

  // FlooNoC AXI interfaces
  floo_terapool_noc_pkg::floo_req_t  [NumX-1:0][NumY-1:0][West:North] floo_axi_req_out,  floo_axi_req_in;
  floo_terapool_noc_pkg::floo_rsp_t  [NumX-1:0][NumY-1:0][West:North] floo_axi_rsp_out,  floo_axi_rsp_in;
  floo_terapool_noc_pkg::floo_wide_t [NumX-1:0][NumY-1:0][West:North] floo_axi_wide_out, floo_axi_wide_in;

  // Chimney configuration
  localparam floo_pkg::chimney_cfg_t ChimneyCfgN = floo_pkg::set_ports(floo_pkg::ChimneyDefaultCfg, 1'b0, 1'b0);
  localparam floo_pkg::chimney_cfg_t ChimneyCfgW = floo_pkg::set_ports(floo_pkg::ChimneyDefaultCfg, 1'b1, 1'b0);

  // Shuffled Physical group[x][y] -> logical group id
  // y=3:  6   2   10  14
  // y=2:  7   3   11  15
  // y=1:  5   1   9   13
  // y=0:  4   0   8   12
  //       x=0 x=1 x=2 x=3
  localparam int unsigned PhysGroupGid [NumX][NumY] = '{
    '{4, 5, 7, 6},      // x=0, y=0..3
    '{0, 1, 3, 2},      // x=1, y=0..3
    '{8, 9, 11, 10},    // x=2, y=0..3
    '{12, 13, 15, 14}   // x=3, y=0..3
  };

  // AXI/HBM physical edge channel mapping.
  // Index is physical edge coordinate.
  // WestHbmId[y]  connects to physical [0][y].West
  // EastHbmId[y]  connects to physical [3][y].East
  // SouthHbmId[x] connects to physical [x][0].South
  // NorthHbmId[x] connects to physical [x][3].North
  // localparam int unsigned WestHbmId  [NumY] = '{4, 5, 7, 6};
  // localparam int unsigned EastHbmId  [NumY] = '{12, 13, 15, 14};
  // localparam int unsigned SouthHbmId [NumX] = '{1, 0, 8, 9};
  // localparam int unsigned NorthHbmId [NumX] = '{3, 2, 10, 11};

  localparam int unsigned WestHbmId [NumY] =
      (NocTopology == 1) ? '{4, 5, 7, 6} : '{0, 1, 2, 3};

  localparam int unsigned EastHbmId [NumY] =
      (NocTopology == 1) ? '{12, 13, 15, 14} : '{12, 13, 14, 15};

  localparam int unsigned SouthHbmId [NumX] =
      (NocTopology == 1) ? '{1, 0, 8, 9} : '{5, 4, 8, 9};

  localparam int unsigned NorthHbmId [NumX] =
      (NocTopology == 1) ? '{3, 2, 10, 11} : '{6, 7, 11, 10};

  // localparam int unsigned PhysicalSlotId = x * NumY + y;
  // localparam int unsigned GroupIdAtSlot  =(NocTopology == 1) ? ShuffledGroupId : PhysicalSlotId;

  for (genvar x = 0; x < NumX; x++) begin : gen_groups_x
    for (genvar y = 0; y < NumY; y++) begin : gen_groups_y

      // localparam int unsigned LogicalGroupId = PhysGroupGid[x][y]; //groupid shuffled with physical placement
      // localparam int unsigned LogicalX   = LogicalGroupId / NumY;
      // localparam int unsigned LogicalY   = LogicalGroupId % NumY;

      localparam int unsigned MeshGroupId     = x * NumY + y;
      localparam int unsigned ShuffledGroupId = PhysGroupGid[x][y];
      localparam int unsigned TcdmGroupId     = (NocTopology == 1) ? ShuffledGroupId : MeshGroupId;

      localparam int unsigned TcdmGroupX = TcdmGroupId / NumY;
      localparam int unsigned TcdmGroupY = TcdmGroupId % NumY;

      group_xy_id_t group_id;
      assign group_id = '{x: TcdmGroupX, y: TcdmGroupY, port_id: 1'b0};

      localparam tcdm_axis_mode_e TcdmEwAdapterMode =(NocTopology == 1) ? (
        (x == 0) ? TCDM_AXIS_SIDE0_EDGE_BRIDGE :
        (x == 1) ? TCDM_AXIS_SWAP_DATA_PASS_FT :
        (x == 2) ? TCDM_AXIS_DEFAULT :
                   TCDM_AXIS_SIDE1_EDGE_BRIDGE
      ) : TCDM_AXIS_DEFAULT;

      localparam tcdm_axis_mode_e TcdmNsAdapterMode =(NocTopology == 1) ? (
        (y == 0) ? TCDM_AXIS_SIDE0_EDGE_BRIDGE :
        (y == 1) ? TCDM_AXIS_DEFAULT :
        (y == 2) ? TCDM_AXIS_SWAP_DATA_PASS_FT :
                   TCDM_AXIS_SIDE1_EDGE_BRIDGE
      ) : TCDM_AXIS_DEFAULT;

      localparam tcdm_adapter_cfg_t TcdmAdapterCfg = '{
        ew: TcdmEwAdapterMode,
        ns: TcdmNsAdapterMode
};


      // TCDM-only shuffled wiring
      if (NocTopology == 1) begin : gen_tcdm_shuffled_torus
        
        // group_xy_id_t group_id;
        // assign group_id = '{x: LogicalX, y: LogicalY, port_id: 1'b0};

        if (x == 0) begin : gen_shuffled_tcdm_rows

          assign floo_tcdm_req_in[0][y][West] = '0;
          assign floo_tcdm_rsp_in[0][y][West] = '0;
          assign ft_tcdm_req_in  [0][y][West] = '0;
          assign ft_tcdm_rsp_in  [0][y][West] = '0;

          assign floo_tcdm_req_in[3][y][East] = '0;
          assign floo_tcdm_rsp_in[3][y][East] = '0;
          assign ft_tcdm_req_in  [3][y][East] = '0;
          assign ft_tcdm_rsp_in  [3][y][East] = '0;

          // ((1,y)router east->)(1,y) West out -> (0,y) East feedthrough (->(0,y)router west in)
          assign ft_tcdm_req_in [0][y][East] = floo_tcdm_req_out[1][y][West];

          // (0,y) East out -> (1,y) feedthrough -> (2,y) West input
          assign ft_tcdm_req_in [1][y][West] = floo_tcdm_req_out[0][y][East];
          assign floo_tcdm_req_in[2][y][West] = ft_tcdm_req_out[1][y][East];

          // (2,y) East out -> (3,y) West input
          assign floo_tcdm_req_in[3][y][West] = floo_tcdm_req_out[2][y][East];

          // (3,y) East out -> (3,y) West feedthrough -> (2,y) feedthrough -> (1,y) West input
          assign ft_tcdm_req_in [2][y][East] = ft_tcdm_req_out[3][y][West];
          assign floo_tcdm_req_in[1][y][East] = ft_tcdm_req_out[2][y][West];

          // Reverse direction links
          assign floo_tcdm_req_in[1][y][West] = ft_tcdm_req_out[0][y][East];

          assign ft_tcdm_req_in [1][y][East] = floo_tcdm_req_out[2][y][West];
          assign floo_tcdm_req_in[0][y][East] = ft_tcdm_req_out[1][y][West];

          assign floo_tcdm_req_in[2][y][East] = floo_tcdm_req_out[3][y][West];

          assign ft_tcdm_req_in [2][y][West] = floo_tcdm_req_out[1][y][East];
          assign ft_tcdm_req_in [3][y][West] = ft_tcdm_req_out[2][y][East];

          // Repeat for resp
          // ((1,y)router east->)(1,y) west out -> (0,y) East feedthrough (->(0,y)router west in)
          assign ft_tcdm_rsp_in [0][y][East] = floo_tcdm_rsp_out[1][y][West];
          // assign floo_tcdm_resp_in[0][y][East] = ft_tcdm_resp_out[0][y][West]; // should in side group level

          // (0,y) East out -> (1,y) feedthrough -> (2,y) West input
          assign ft_tcdm_rsp_in [1][y][West] = floo_tcdm_rsp_out[0][y][East];
          assign floo_tcdm_rsp_in[2][y][West] = ft_tcdm_rsp_out[1][y][East];

          // (2,y) East out -> (3,y) West input
          assign floo_tcdm_rsp_in[3][y][West] = floo_tcdm_rsp_out[2][y][East];

          // (3,y) East out -> (3,y) feedthrough -> (2,y) feedthrough -> (1,y) West input
          assign ft_tcdm_rsp_in [2][y][East] = ft_tcdm_rsp_out[3][y][West];
          assign floo_tcdm_rsp_in[1][y][East] = ft_tcdm_rsp_out[2][y][West];

          // Reverse direction links
          assign floo_tcdm_rsp_in[1][y][West] = ft_tcdm_rsp_out[0][y][East];

          assign ft_tcdm_rsp_in [1][y][East] = floo_tcdm_rsp_out[2][y][West];
          assign floo_tcdm_rsp_in[0][y][East] = ft_tcdm_rsp_out[1][y][West];

          assign floo_tcdm_rsp_in[2][y][East] = floo_tcdm_rsp_out[3][y][West];

          assign ft_tcdm_rsp_in [2][y][West] = floo_tcdm_rsp_out[1][y][East];
          assign ft_tcdm_rsp_in [3][y][West] = ft_tcdm_rsp_out[2][y][East];
        end
        
        if (y == 0) begin : gen_shuffled_tcdm_cols

          assign floo_tcdm_req_in[x][0][South] = '0;
          assign floo_tcdm_rsp_in[x][0][South] = '0;
          assign ft_tcdm_req_in  [x][0][South] = '0;
          assign ft_tcdm_rsp_in  [x][0][South] = '0;

          assign floo_tcdm_req_in[x][3][North] = '0;
          assign floo_tcdm_rsp_in[x][3][North] = '0;
          assign ft_tcdm_req_in  [x][3][North] = '0;
          assign ft_tcdm_rsp_in  [x][3][North] = '0;

          // ((x,2) router south out->)(x,2) North out -> (x,3) South feedthrough (-> (x,3) North router input)
          assign ft_tcdm_req_in [x][3][South] = floo_tcdm_req_out[x][2][North];

          // (x,3) South out -> (x,2) feedthrough -> (x,1) North input
          assign ft_tcdm_req_in [x][2][North] = floo_tcdm_req_out[x][3][South];
          assign floo_tcdm_req_in[x][1][North] = ft_tcdm_req_out[x][2][South];

          // (x,1) South out -> (x,0) North input
          assign floo_tcdm_req_in[x][0][North] = floo_tcdm_req_out[x][1][South];

          // ((x,0) South router out->) (x,0) north feedthrough -> (x,1) feedthrough -> (x,2) South input(-> (x,2) North router in)
          assign ft_tcdm_req_in [x][1][South] = ft_tcdm_req_out[x][0][North];
          assign floo_tcdm_req_in[x][2][South] = ft_tcdm_req_out[x][1][North];

          // Reverse direction links need the analogous North-out paths.
          assign floo_tcdm_req_in[x][2][North] = ft_tcdm_req_out[x][3][South];

          assign floo_tcdm_req_in[x][3][South] = ft_tcdm_req_out[x][2][North];
          assign ft_tcdm_req_in[x][2][South] = floo_tcdm_req_out[x][1][North];

          assign floo_tcdm_req_in[x][1][South] = floo_tcdm_req_out[x][0][North];

          assign ft_tcdm_req_in[x][1][North] = floo_tcdm_req_out[x][2][South];
          assign ft_tcdm_req_in[x][0][North] = ft_tcdm_req_out[x][1][South];


          // TCDM resp
          // (x,2) South out -> (x,3) South feedthrough -> (x,3) North input
          assign ft_tcdm_rsp_in [x][3][South] = floo_tcdm_rsp_out[x][2][North];

          // (x,3) South out -> (x,2) feedthrough -> (x,1) North input
          assign ft_tcdm_rsp_in [x][2][North] = floo_tcdm_rsp_out[x][3][South];
          assign floo_tcdm_rsp_in[x][1][North] = ft_tcdm_rsp_out[x][2][South];

          // (x,1) South out -> (x,0) North input
          assign floo_tcdm_rsp_in[x][0][North] = floo_tcdm_rsp_out[x][1][South];

          // (x,0) South out -> (x,0) feedthrough -> (x,1) feedthrough -> (x,2) South input
          assign ft_tcdm_rsp_in [x][1][South] = ft_tcdm_rsp_out[x][0][North];
          assign floo_tcdm_rsp_in[x][2][South] = ft_tcdm_rsp_out[x][1][North];

          // Reverse direction links need the analogous North-out paths.
          assign floo_tcdm_rsp_in[x][2][North] = ft_tcdm_rsp_out[x][3][South];

          assign floo_tcdm_rsp_in[x][3][South] = ft_tcdm_rsp_out[x][2][North];
          assign ft_tcdm_rsp_in[x][2][South] = floo_tcdm_rsp_out[x][1][North];

          assign floo_tcdm_rsp_in[x][1][South] = floo_tcdm_rsp_out[x][0][North];

          assign ft_tcdm_rsp_in[x][1][North] = floo_tcdm_rsp_out[x][2][South];
          assign ft_tcdm_rsp_in[x][0][North] = ft_tcdm_rsp_out[x][1][South];
        end
      end else begin : gen_tcdm_mesh
        // group_xy_id_t group_id;
        // assign group_id = '{x:x, y:y, port_id:1'b0};
        // old mesh TCDM horizontal assignments
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

        // old mesh TCDM vertical assignments
        if (y == 0) begin
          assign floo_tcdm_req_in[x][y][South] = '0;
          assign floo_tcdm_rsp_in[x][y][South] = '0;
          assign floo_tcdm_req_in[x][y][North] = floo_tcdm_req_out[x][y+1][South];
          assign floo_tcdm_rsp_in[x][y][North] = floo_tcdm_rsp_out[x][y+1][South];
        end else if (y == NumY-1) begin
          assign floo_tcdm_req_in[x][y][North] = '0;
          assign floo_tcdm_rsp_in[x][y][North] = '0;
          assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][y-1][North];
          assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][y-1][North];
        end else begin
          assign floo_tcdm_req_in[x][y][North] = floo_tcdm_req_out[x][y+1][South];
          assign floo_tcdm_rsp_in[x][y][North] = floo_tcdm_rsp_out[x][y+1][South];
          assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][y-1][North];
          assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][y-1][North];
        end
      end

      // AXI-only physical mesh/chimney wiring
      if (x == 0) begin : gen_hbm_chimney_west
        // // West
        // if (NocTopology == 1) begin
        //   assign floo_tcdm_req_in[x][y][West] = floo_tcdm_req_out[NumX-1][y][East];
        //   assign floo_tcdm_rsp_in[x][y][West] = floo_tcdm_rsp_out[NumX-1][y][East];
        // end else begin
        //   assign floo_tcdm_req_in[x][y][West] = '0;
        //   assign floo_tcdm_rsp_in[x][y][West] = '0;
        // end

        // // East
        // assign floo_tcdm_req_in[x][y][East] = floo_tcdm_req_out[x+1][y][West];
        // assign floo_tcdm_rsp_in[x][y][East] = floo_tcdm_rsp_out[x+1][y][West];

        // AXI East from interior
        assign floo_axi_req_in[x][y][East]  = floo_axi_req_out[x+1][y][West];
        assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_out[x+1][y][West];
        assign floo_axi_wide_in[x][y][East] = floo_axi_wide_out[x+1][y][West];

        // // AXI West
        // assign floo_axi_req_in[x][y][West]  = floo_axi_req_i[y];
        // assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_i[y];
        // assign floo_axi_wide_in[x][y][West] = floo_axi_wide_i[y];
        // assign floo_axi_wide_o[y]           = floo_axi_wide_out[x][y][West];
        // assign floo_axi_req_o[y]            = floo_axi_req_out[x][y][West];
        // assign floo_axi_rsp_o[y]            = floo_axi_rsp_out[x][y][West];
        
        // AXI West dege
        localparam int unsigned HbmWestId = WestHbmId[y];

        assign floo_axi_req_in[x][y][West]  = floo_axi_req_i[HbmWestId];
        assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_i[HbmWestId];
        assign floo_axi_wide_in[x][y][West] = floo_axi_wide_i[HbmWestId];

        assign floo_axi_wide_o[HbmWestId] = floo_axi_wide_out[x][y][West];
        assign floo_axi_req_o[HbmWestId]  = floo_axi_req_out[x][y][West];
        assign floo_axi_rsp_o[HbmWestId]  = floo_axi_rsp_out[x][y][West];

      end else if (x == NumX-1) begin : gen_hbm_chimney_east
        // // East
        // if (NocTopology == 1) begin
        //   assign floo_tcdm_req_in[x][y][East] = floo_tcdm_req_out[0][y][West];
        //   assign floo_tcdm_rsp_in[x][y][East] = floo_tcdm_rsp_out[0][y][West];
        // end else begin
        //   assign floo_tcdm_req_in[x][y][East] = '0;
        //   assign floo_tcdm_rsp_in[x][y][East] = '0;
        // end

        // // West
        // assign floo_tcdm_req_in[x][y][West] = floo_tcdm_req_out[x-1][y][East];
        // assign floo_tcdm_rsp_in[x][y][West] = floo_tcdm_rsp_out[x-1][y][East];

        // AXI West from interior
        assign floo_axi_req_in[x][y][West]  = floo_axi_req_out[x-1][y][East];
        assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_out[x-1][y][East];
        assign floo_axi_wide_in[x][y][West] = floo_axi_wide_out[x-1][y][East];

        // // AXI East
        // assign floo_axi_req_in[x][y][East]  = floo_axi_req_i[y+12];
        // assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_i[y+12];
        // assign floo_axi_wide_in[x][y][East] = floo_axi_wide_i[y+12];
        // assign floo_axi_wide_o[y+12]        = floo_axi_wide_out[x][y][East];
        // assign floo_axi_req_o[y+12]         = floo_axi_req_out[x][y][East];
        // assign floo_axi_rsp_o[y+12]         = floo_axi_rsp_out[x][y][East];

        // AXI East edge
        localparam int unsigned HbmEastId = EastHbmId[y];
        assign floo_axi_req_in[x][y][East]  = floo_axi_req_i[HbmEastId];
        assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_i[HbmEastId];
        assign floo_axi_wide_in[x][y][East] = floo_axi_wide_i[HbmEastId];

        assign floo_axi_wide_o[HbmEastId] = floo_axi_wide_out[x][y][East];
        assign floo_axi_req_o[HbmEastId]  = floo_axi_req_out[x][y][East];
        assign floo_axi_rsp_o[HbmEastId]  = floo_axi_rsp_out[x][y][East];

      end else begin : gen_hor_connections
        // East
        // assign floo_tcdm_req_in[x][y][East] = floo_tcdm_req_out[x+1][y][West];
        // assign floo_tcdm_rsp_in[x][y][East] = floo_tcdm_rsp_out[x+1][y][West];
        assign floo_axi_req_in[x][y][East]  = floo_axi_req_out[x+1][y][West];
        assign floo_axi_rsp_in[x][y][East]  = floo_axi_rsp_out[x+1][y][West];
        assign floo_axi_wide_in[x][y][East] = floo_axi_wide_out[x+1][y][West];

        // West
        // assign floo_tcdm_req_in[x][y][West] = floo_tcdm_req_out[x-1][y][East];
        // assign floo_tcdm_rsp_in[x][y][West] = floo_tcdm_rsp_out[x-1][y][East];
        assign floo_axi_req_in[x][y][West]  = floo_axi_req_out[x-1][y][East];
        assign floo_axi_rsp_in[x][y][West]  = floo_axi_rsp_out[x-1][y][East];
        assign floo_axi_wide_in[x][y][West] = floo_axi_wide_out[x-1][y][East];
      end
    
      if (y == 0) begin : gen_hbm_chimney_south
        // // South
        // if (NocTopology == 1) begin
        //   assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][NumY-1][North];
        //   assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][NumY-1][North];
        // end else begin
        //   assign floo_tcdm_req_in[x][y][South] = '0;
        //   assign floo_tcdm_rsp_in[x][y][South] = '0;
        // end

        // // North
        // assign floo_tcdm_req_in [x][y][North] = floo_tcdm_req_out [x][y+1][South];
        // assign floo_tcdm_rsp_in [x][y][North] = floo_tcdm_rsp_out [x][y+1][South];

        // AXI North from interior
        assign floo_axi_req_in  [x][y][North] = floo_axi_req_out  [x][y+1][South];
        assign floo_axi_rsp_in  [x][y][North] = floo_axi_rsp_out  [x][y+1][South];
        assign floo_axi_wide_in [x][y][North] = floo_axi_wide_out [x][y+1][South];

        // AXI South edge
        localparam int unsigned HbmSouthId = SouthHbmId[x];

        assign floo_axi_req_in[x][y][South]  = floo_axi_req_i[HbmSouthId];
        assign floo_axi_rsp_in[x][y][South]  = floo_axi_rsp_i[HbmSouthId];
        assign floo_axi_wide_in[x][y][South] = floo_axi_wide_i[HbmSouthId];

        assign floo_axi_wide_o[HbmSouthId] = floo_axi_wide_out[x][y][South];
        assign floo_axi_req_o[HbmSouthId]  = floo_axi_req_out[x][y][South];
        assign floo_axi_rsp_o[HbmSouthId]  = floo_axi_rsp_out[x][y][South];

        // if (x < NumX/2) begin : gen_normal_chimneys
        //   // AXI South
        //   assign floo_axi_req_in[x][y][South]  = floo_axi_req_i[5-x];
        //   assign floo_axi_rsp_in[x][y][South]  = floo_axi_rsp_i[5-x];
        //   assign floo_axi_wide_in[x][y][South] = floo_axi_wide_i[5-x];
        //   assign floo_axi_wide_o[5-x]          = floo_axi_wide_out[x][y][South];
        //   assign floo_axi_req_o[5-x]           = floo_axi_req_out[x][y][South];
        //   assign floo_axi_rsp_o[5-x]           = floo_axi_rsp_out[x][y][South];

        // end else begin : gen_normal_chimneys_2
        //   // AXI South
        //   assign floo_axi_req_in[x][y][South]  = floo_axi_req_i[x+6];
        //   assign floo_axi_rsp_in[x][y][South]  = floo_axi_rsp_i[x+6];
        //   assign floo_axi_wide_in[x][y][South] = floo_axi_wide_i[x+6];
        //   assign floo_axi_wide_o[x+6]          = floo_axi_wide_out[x][y][South];
        //   assign floo_axi_req_o[x+6]           = floo_axi_req_out[x][y][South];
        //   assign floo_axi_rsp_o[x+6]           = floo_axi_rsp_out[x][y][South];
        // end

      end else if (y == NumY-1) begin
        // TCDM North
        // if (NocTopology == 1) begin
        //   assign floo_tcdm_req_in[x][y][North] = floo_tcdm_req_out[x][0][South];
        //   assign floo_tcdm_rsp_in[x][y][North] = floo_tcdm_rsp_out[x][0][South];
        // end else begin
        //   assign floo_tcdm_req_in[x][y][North] = '0;
        //   assign floo_tcdm_rsp_in[x][y][North] = '0;
        // end

        // // TCDM South
        // assign floo_tcdm_req_in[x][y][South] = floo_tcdm_req_out[x][y-1][North];
        // assign floo_tcdm_rsp_in[x][y][South] = floo_tcdm_rsp_out[x][y-1][North];

        // AXI South from interior
        assign floo_axi_req_in [x][y][South] = floo_axi_req_out [x][y-1][North];
        assign floo_axi_rsp_in [x][y][South] = floo_axi_rsp_out [x][y-1][North];
        assign floo_axi_wide_in[x][y][South] = floo_axi_wide_out[x][y-1][North];

        //AXI north edge
        localparam int unsigned HbmNorthId = NorthHbmId[x];

        assign floo_axi_req_in[x][y][North]  = floo_axi_req_i[HbmNorthId];
        assign floo_axi_rsp_in[x][y][North]  = floo_axi_rsp_i[HbmNorthId];
        assign floo_axi_wide_in[x][y][North] = floo_axi_wide_i[HbmNorthId];

        assign floo_axi_wide_o[HbmNorthId] = floo_axi_wide_out[x][y][North];
        assign floo_axi_req_o[HbmNorthId]  = floo_axi_req_out[x][y][North];
        assign floo_axi_rsp_o[HbmNorthId]  = floo_axi_rsp_out[x][y][North];

        // if (x < NumX/2) begin
        //   // AXI North
        //   assign floo_axi_req_in [x][y][North] = floo_axi_req_i [x+6];
        //   assign floo_axi_rsp_in [x][y][North] = floo_axi_rsp_i [x+6];
        //   assign floo_axi_wide_in[x][y][North] = floo_axi_wide_i[x+6];
        //   assign floo_axi_wide_o [x+6]         = floo_axi_wide_out[x][y][North];
        //   assign floo_axi_req_o  [x+6]         = floo_axi_req_out [x][y][North];
        //   assign floo_axi_rsp_o  [x+6]         = floo_axi_rsp_out [x][y][North];
        // end else begin
        //   // AXI North
        //   assign floo_axi_req_in [x][y][North] = floo_axi_req_i [13-x];
        //   assign floo_axi_rsp_in [x][y][North] = floo_axi_rsp_i [13-x];
        //   assign floo_axi_wide_in[x][y][North] = floo_axi_wide_i[13-x];
        //   assign floo_axi_wide_o [13-x]        = floo_axi_wide_out[x][y][North];
        //   assign floo_axi_req_o  [13-x]        = floo_axi_req_out [x][y][North];
        //   assign floo_axi_rsp_o  [13-x]        = floo_axi_rsp_out [x][y][North];
        // end

      end else begin
        // North
        // assign floo_tcdm_req_in [x][y][North] = floo_tcdm_req_out [x][y+1][South];
        // assign floo_tcdm_rsp_in [x][y][North] = floo_tcdm_rsp_out [x][y+1][South];
        assign floo_axi_req_in  [x][y][North] = floo_axi_req_out  [x][y+1][South];
        assign floo_axi_rsp_in  [x][y][North] = floo_axi_rsp_out  [x][y+1][South];
        assign floo_axi_wide_in [x][y][North] = floo_axi_wide_out [x][y+1][South];

        // South
        // assign floo_tcdm_req_in [x][y][South] = floo_tcdm_req_out [x][y-1][North];
        // assign floo_tcdm_rsp_in [x][y][South] = floo_tcdm_rsp_out [x][y-1][North];
        assign floo_axi_req_in  [x][y][South] = floo_axi_req_out  [x][y-1][North];
        assign floo_axi_rsp_in  [x][y][South] = floo_axi_rsp_out  [x][y-1][North];
        assign floo_axi_wide_in [x][y][South] = floo_axi_wide_out [x][y-1][North];
      end

      if (PostLayoutGr & (x == 0) & (y == 0)) begin : gen_postly_group
        mempool_group_floonoc_wrapper_postlayout i_group (
          .clk_i              (clk_i),
          .rst_ni             (rst_ni),
          .testmode_i         (testmode_i),
          .scan_enable_i      (scan_enable_i),
          .scan_data_i        (/* Unconnected */),
          .scan_data_o        (/* Unconnected */),
          .group_id_i         (group_id_t'({group_id.x, group_id.y})),
          // .floo_id_i          (id_t'(GroupX0Y0 + x*NumY + y)),
          // .route_table_i      (floo_terapool_noc_pkg::RoutingTables[GroupX0Y0 + x*NumY + y]),
          .floo_id_i          (id_t'(GroupX0Y0 + TcdmGroupId)),
          .route_table_i      (floo_terapool_noc_pkg::RoutingTables[GroupX0Y0 + TcdmGroupId]),
          // TCDM Router interface
          .floo_tcdm_req_o    (floo_tcdm_req_out[x][y]),
          .floo_tcdm_rsp_o    (floo_tcdm_rsp_out[x][y]),
          .floo_tcdm_req_i    (floo_tcdm_req_in[x][y]),
          .floo_tcdm_rsp_i    (floo_tcdm_rsp_in[x][y]),
          // // TCDM NoC feedthrough
          // .ft_tcdm_req_i   (ft_tcdm_req_in [x][y]),
          // .ft_tcdm_req_o   (ft_tcdm_req_out[x][y]),
          // .ft_tcdm_rsp_i   (ft_tcdm_rsp_in [x][y]),
          // .ft_tcdm_rsp_o   (ft_tcdm_rsp_out[x][y]),
          .wake_up_i          (wake_up_q[(NumY*x+y)*NumCoresPerGroup +: NumCoresPerGroup]),
          .ro_cache_ctrl_i    (ro_cache_ctrl_q[(NumY*x+y)]),
          // DMA request
          // .dma_req_i          (dma_req_group_q[(NumY*x+y)]),
          // .dma_req_valid_i    (dma_req_group_q_valid[(NumY*x+y)]),
          // .dma_req_ready_o    (dma_req_group_q_ready[(NumY*x+y)]),
          .dma_req_i       (dma_req_group_q[TcdmGroupId]),
          .dma_req_valid_i (dma_req_group_q_valid[TcdmGroupId]),
          .dma_req_ready_o (dma_req_group_q_ready[TcdmGroupId]),
          // DMA status
          .dma_meta_o_backend_idle_   (dma_meta[(NumY*x+y)][1]),
          .dma_meta_o_trans_complete_ (dma_meta[(NumY*x+y)][0]),
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
          // .floo_id_i          (id_t'(GroupX0Y0 + x*NumY + y)),
          // .route_table_i      (floo_terapool_noc_pkg::RoutingTables[GroupX0Y0 + x*NumY + y]),
          .floo_id_i          (id_t'(GroupX0Y0 + TcdmGroupId)),
          .route_table_i      (floo_terapool_noc_pkg::RoutingTables[GroupX0Y0 + TcdmGroupId]),
          // TCDM Router interface
          .floo_tcdm_req_o    (floo_tcdm_req_out[x][y]),
          .floo_tcdm_rsp_o    (floo_tcdm_rsp_out[x][y]),
          .floo_tcdm_req_i    (floo_tcdm_req_in[x][y]),
          .floo_tcdm_rsp_i    (floo_tcdm_rsp_in[x][y]),
          // TCDM NoC feedthrough
          .ft_tcdm_req_i      (ft_tcdm_req_in [x][y]),
          .ft_tcdm_req_o      (ft_tcdm_req_out[x][y]),
          .ft_tcdm_rsp_i      (ft_tcdm_rsp_in [x][y]),
          .ft_tcdm_rsp_o      (ft_tcdm_rsp_out[x][y]),
          .tcdm_adapter_cfg_i (TcdmAdapterCfg),
          // .wake_up_i          (wake_up_q[(NumY*x+y)*NumCoresPerGroup +: NumCoresPerGroup]),
          // .ro_cache_ctrl_i    (ro_cache_ctrl_q[(NumY*x+y)]),
          .wake_up_i       (wake_up_q[TcdmGroupId*NumCoresPerGroup +: NumCoresPerGroup]),
          .ro_cache_ctrl_i (ro_cache_ctrl_q[TcdmGroupId]),
          // DMA request
          // .dma_req_i          (dma_req_group_q[(NumY*x+y)]),
          // .dma_req_valid_i    (dma_req_group_q_valid[(NumY*x+y)]),
          // .dma_req_ready_o    (dma_req_group_q_ready[(NumY*x+y)]),
          .dma_req_i       (dma_req_group_q[TcdmGroupId]),
          .dma_req_valid_i (dma_req_group_q_valid[TcdmGroupId]),
          .dma_req_ready_o (dma_req_group_q_ready[TcdmGroupId]),
          // DMA status
          // .dma_meta_o         (dma_meta[(NumY*x+y)]),
          .dma_meta_o      (dma_meta[TcdmGroupId]),
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
