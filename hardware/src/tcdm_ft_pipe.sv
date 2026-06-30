module tcdm_ft_pipe #(
  parameter type payload_t = logic,
  parameter int unsigned NumStages = 2
) (
  input  logic     clk_i,
  input  logic     rst_ni,

  input  payload_t data_i,
  input  logic     valid_i,
  output logic     ready_o,

  output payload_t data_o,
  output logic     valid_o,
  input  logic     ready_i
);

  payload_t [NumStages:0] data;
  logic     [NumStages:0] valid;
  logic     [NumStages:0] ready;

  assign data[0] = data_i;
  assign valid[0] = valid_i;
  assign ready_o = ready[0];

  assign data_o = data[NumStages];
  assign valid_o = valid[NumStages];
  assign ready[NumStages] = ready_i;

  for (genvar s = 0; s < NumStages; s++) begin : gen_stage
    spill_register #(
      .T      (payload_t),
      .Bypass (1'b0)
    ) i_spill_register (
      .clk_i,
      .rst_ni,
      .valid_i (valid[s]),
      .ready_o (ready[s]),
      .data_i  (data[s]),
      .valid_o (valid[s+1]),
      .ready_i (ready[s+1]),
      .data_o  (data[s+1])
    );
  end

endmodule
