//
// Generated by Bluespec Compiler, version 2023.01 (build 52adafa5)
//
// timestamp removed
//
// BVI format method schedule info:
// schedule ifc_startSwitching  CF ( ifc_printSwStats,
// 				  ifc_start_shoal,
// 				  inverseIfc_startSwitching,
// 				  inverseIfc_printSwStats,
// 				  inverseIfc_start_shoal );
// schedule ifc_startSwitching  C ( ifc_startSwitching );
//
// schedule ifc_printSwStats  CF ( ifc_startSwitching,
// 				ifc_start_shoal,
// 				inverseIfc_startSwitching,
// 				inverseIfc_printSwStats,
// 				inverseIfc_start_shoal );
// schedule ifc_printSwStats  C ( ifc_printSwStats );
//
// schedule ifc_start_shoal  CF ( ifc_startSwitching,
// 			       ifc_printSwStats,
// 			       inverseIfc_startSwitching,
// 			       inverseIfc_printSwStats,
// 			       inverseIfc_start_shoal );
// schedule ifc_start_shoal  C ( ifc_start_shoal );
//
// schedule inverseIfc_startSwitching  CF ( ifc_startSwitching,
// 					 ifc_printSwStats,
// 					 ifc_start_shoal,
// 					 inverseIfc_printSwStats,
// 					 inverseIfc_start_shoal );
// schedule inverseIfc_startSwitching  C ( inverseIfc_startSwitching );
//
// schedule inverseIfc_printSwStats  CF ( ifc_startSwitching,
// 				       ifc_printSwStats,
// 				       ifc_start_shoal,
// 				       inverseIfc_startSwitching,
// 				       inverseIfc_start_shoal );
// schedule inverseIfc_printSwStats  C ( inverseIfc_printSwStats );
//
// schedule inverseIfc_start_shoal  CF ( ifc_startSwitching,
// 				      ifc_printSwStats,
// 				      ifc_start_shoal,
// 				      inverseIfc_startSwitching,
// 				      inverseIfc_printSwStats );
// schedule inverseIfc_start_shoal  C ( inverseIfc_start_shoal );
//
//
// Ports:
// Name                         I/O  size props
// RDY_ifc_startSwitching         O     1 reg
// RDY_ifc_printSwStats           O     1 reg
// RDY_ifc_start_shoal            O     1 reg
// inverseIfc_startSwitching      O    72 reg
// RDY_inverseIfc_startSwitching  O     1 reg
// inverseIfc_printSwStats        O    32 reg
// RDY_inverseIfc_printSwStats    O     1 reg
// inverseIfc_start_shoal         O   120 reg
// RDY_inverseIfc_start_shoal     O     1 reg
// CLK                            I     1 clock
// RST_N                          I     1 reset
// ifc_startSwitching_reconfig_flag  I     8 reg
// ifc_startSwitching_timeslot    I    64 reg
// ifc_start_shoal_idx            I    32 reg
// ifc_start_shoal_rate           I    16 reg
// ifc_start_shoal_timeslot       I     8 reg
// ifc_start_shoal_cycles         I    64 reg
// EN_ifc_startSwitching          I     1
// EN_ifc_printSwStats            I     1
// EN_ifc_start_shoal             I     1
// EN_inverseIfc_startSwitching   I     1
// EN_inverseIfc_printSwStats     I     1
// EN_inverseIfc_start_shoal      I     1
//
// No combinational paths from inputs to outputs
//
//

`ifdef BSV_ASSIGNMENT_DELAY
`else
  `define BSV_ASSIGNMENT_DELAY
`endif

`ifdef BSV_POSITIVE_RESET
  `define BSV_RESET_VALUE 1'b1
  `define BSV_RESET_EDGE posedge
`else
  `define BSV_RESET_VALUE 1'b0
  `define BSV_RESET_EDGE negedge
`endif

module mkShoalMultiSimTopRequestInverter(CLK,
					 RST_N,

					 ifc_startSwitching_reconfig_flag,
					 ifc_startSwitching_timeslot,
					 EN_ifc_startSwitching,
					 RDY_ifc_startSwitching,

					 EN_ifc_printSwStats,
					 RDY_ifc_printSwStats,

					 ifc_start_shoal_idx,
					 ifc_start_shoal_rate,
					 ifc_start_shoal_timeslot,
					 ifc_start_shoal_cycles,
					 EN_ifc_start_shoal,
					 RDY_ifc_start_shoal,

					 EN_inverseIfc_startSwitching,
					 inverseIfc_startSwitching,
					 RDY_inverseIfc_startSwitching,

					 EN_inverseIfc_printSwStats,
					 inverseIfc_printSwStats,
					 RDY_inverseIfc_printSwStats,

					 EN_inverseIfc_start_shoal,
					 inverseIfc_start_shoal,
					 RDY_inverseIfc_start_shoal);
  input  CLK;
  input  RST_N;

  // action method ifc_startSwitching
  input  [7 : 0] ifc_startSwitching_reconfig_flag;
  input  [63 : 0] ifc_startSwitching_timeslot;
  input  EN_ifc_startSwitching;
  output RDY_ifc_startSwitching;

  // action method ifc_printSwStats
  input  EN_ifc_printSwStats;
  output RDY_ifc_printSwStats;

  // action method ifc_start_shoal
  input  [31 : 0] ifc_start_shoal_idx;
  input  [15 : 0] ifc_start_shoal_rate;
  input  [7 : 0] ifc_start_shoal_timeslot;
  input  [63 : 0] ifc_start_shoal_cycles;
  input  EN_ifc_start_shoal;
  output RDY_ifc_start_shoal;

  // actionvalue method inverseIfc_startSwitching
  input  EN_inverseIfc_startSwitching;
  output [71 : 0] inverseIfc_startSwitching;
  output RDY_inverseIfc_startSwitching;

  // actionvalue method inverseIfc_printSwStats
  input  EN_inverseIfc_printSwStats;
  output [31 : 0] inverseIfc_printSwStats;
  output RDY_inverseIfc_printSwStats;

  // actionvalue method inverseIfc_start_shoal
  input  EN_inverseIfc_start_shoal;
  output [119 : 0] inverseIfc_start_shoal;
  output RDY_inverseIfc_start_shoal;

  // signals for module outputs
  wire [119 : 0] inverseIfc_start_shoal;
  wire [71 : 0] inverseIfc_startSwitching;
  wire [31 : 0] inverseIfc_printSwStats;
  wire RDY_ifc_printSwStats,
       RDY_ifc_startSwitching,
       RDY_ifc_start_shoal,
       RDY_inverseIfc_printSwStats,
       RDY_inverseIfc_startSwitching,
       RDY_inverseIfc_start_shoal;

  // ports of submodule fifo_printSwStats
  wire [31 : 0] fifo_printSwStats_D_IN, fifo_printSwStats_D_OUT;
  wire fifo_printSwStats_CLR,
       fifo_printSwStats_DEQ,
       fifo_printSwStats_EMPTY_N,
       fifo_printSwStats_ENQ,
       fifo_printSwStats_FULL_N;

  // ports of submodule fifo_startSwitching
  wire [71 : 0] fifo_startSwitching_D_IN, fifo_startSwitching_D_OUT;
  wire fifo_startSwitching_CLR,
       fifo_startSwitching_DEQ,
       fifo_startSwitching_EMPTY_N,
       fifo_startSwitching_ENQ,
       fifo_startSwitching_FULL_N;

  // ports of submodule fifo_start_shoal
  wire [119 : 0] fifo_start_shoal_D_IN, fifo_start_shoal_D_OUT;
  wire fifo_start_shoal_CLR,
       fifo_start_shoal_DEQ,
       fifo_start_shoal_EMPTY_N,
       fifo_start_shoal_ENQ,
       fifo_start_shoal_FULL_N;

  // action method ifc_startSwitching
  assign RDY_ifc_startSwitching = fifo_startSwitching_FULL_N ;

  // action method ifc_printSwStats
  assign RDY_ifc_printSwStats = fifo_printSwStats_FULL_N ;

  // action method ifc_start_shoal
  assign RDY_ifc_start_shoal = fifo_start_shoal_FULL_N ;

  // actionvalue method inverseIfc_startSwitching
  assign inverseIfc_startSwitching = fifo_startSwitching_D_OUT ;
  assign RDY_inverseIfc_startSwitching = fifo_startSwitching_EMPTY_N ;

  // actionvalue method inverseIfc_printSwStats
  assign inverseIfc_printSwStats = fifo_printSwStats_D_OUT ;
  assign RDY_inverseIfc_printSwStats = fifo_printSwStats_EMPTY_N ;

  // actionvalue method inverseIfc_start_shoal
  assign inverseIfc_start_shoal = fifo_start_shoal_D_OUT ;
  assign RDY_inverseIfc_start_shoal = fifo_start_shoal_EMPTY_N ;

  // submodule fifo_printSwStats
  FIFO2 #(.width(32'd32), .guarded(1'd1)) fifo_printSwStats(.RST(RST_N),
							    .CLK(CLK),
							    .D_IN(fifo_printSwStats_D_IN),
							    .ENQ(fifo_printSwStats_ENQ),
							    .DEQ(fifo_printSwStats_DEQ),
							    .CLR(fifo_printSwStats_CLR),
							    .D_OUT(fifo_printSwStats_D_OUT),
							    .FULL_N(fifo_printSwStats_FULL_N),
							    .EMPTY_N(fifo_printSwStats_EMPTY_N));

  // submodule fifo_startSwitching
  FIFO2 #(.width(32'd72), .guarded(1'd1)) fifo_startSwitching(.RST(RST_N),
							      .CLK(CLK),
							      .D_IN(fifo_startSwitching_D_IN),
							      .ENQ(fifo_startSwitching_ENQ),
							      .DEQ(fifo_startSwitching_DEQ),
							      .CLR(fifo_startSwitching_CLR),
							      .D_OUT(fifo_startSwitching_D_OUT),
							      .FULL_N(fifo_startSwitching_FULL_N),
							      .EMPTY_N(fifo_startSwitching_EMPTY_N));

  // submodule fifo_start_shoal
  FIFO2 #(.width(32'd120), .guarded(1'd1)) fifo_start_shoal(.RST(RST_N),
							    .CLK(CLK),
							    .D_IN(fifo_start_shoal_D_IN),
							    .ENQ(fifo_start_shoal_ENQ),
							    .DEQ(fifo_start_shoal_DEQ),
							    .CLR(fifo_start_shoal_CLR),
							    .D_OUT(fifo_start_shoal_D_OUT),
							    .FULL_N(fifo_start_shoal_FULL_N),
							    .EMPTY_N(fifo_start_shoal_EMPTY_N));

  // submodule fifo_printSwStats
  assign fifo_printSwStats_D_IN = 32'd0 ;
  assign fifo_printSwStats_ENQ = EN_ifc_printSwStats ;
  assign fifo_printSwStats_DEQ = EN_inverseIfc_printSwStats ;
  assign fifo_printSwStats_CLR = 1'b0 ;

  // submodule fifo_startSwitching
  assign fifo_startSwitching_D_IN =
	     { ifc_startSwitching_reconfig_flag,
	       ifc_startSwitching_timeslot } ;
  assign fifo_startSwitching_ENQ = EN_ifc_startSwitching ;
  assign fifo_startSwitching_DEQ = EN_inverseIfc_startSwitching ;
  assign fifo_startSwitching_CLR = 1'b0 ;

  // submodule fifo_start_shoal
  assign fifo_start_shoal_D_IN =
	     { ifc_start_shoal_idx,
	       ifc_start_shoal_rate,
	       ifc_start_shoal_timeslot,
	       ifc_start_shoal_cycles } ;
  assign fifo_start_shoal_ENQ = EN_ifc_start_shoal ;
  assign fifo_start_shoal_DEQ = EN_inverseIfc_start_shoal ;
  assign fifo_start_shoal_CLR = 1'b0 ;
endmodule  // mkShoalMultiSimTopRequestInverter

