//
// Generated by Bluespec Compiler, version 2023.01 (build 52adafa5)
//
// timestamp removed
//
// BVI format method schedule info:
// schedule portalIfc_messageSize_size  CF ( portalIfc_messageSize_size,
// 					  portalIfc_indications_0_first,
// 					  portalIfc_indications_0_deq,
// 					  portalIfc_indications_0_notEmpty,
// 					  portalIfc_indications_1_first,
// 					  portalIfc_indications_1_deq,
// 					  portalIfc_indications_1_notEmpty,
// 					  portalIfc_indications_2_first,
// 					  portalIfc_indications_2_deq,
// 					  portalIfc_indications_2_notEmpty,
// 					  portalIfc_intr_status,
// 					  portalIfc_intr_channel,
// 					  ifc_startSwitching,
// 					  ifc_printSwStats,
// 					  ifc_start_shoal );
//
// schedule portalIfc_indications_0_first  CF ( portalIfc_messageSize_size,
// 					     portalIfc_indications_0_first,
// 					     portalIfc_indications_0_notEmpty,
// 					     portalIfc_indications_1_first,
// 					     portalIfc_indications_1_deq,
// 					     portalIfc_indications_1_notEmpty,
// 					     portalIfc_indications_2_first,
// 					     portalIfc_indications_2_deq,
// 					     portalIfc_indications_2_notEmpty,
// 					     portalIfc_intr_status,
// 					     portalIfc_intr_channel,
// 					     ifc_startSwitching,
// 					     ifc_printSwStats,
// 					     ifc_start_shoal );
// schedule portalIfc_indications_0_first  SB ( portalIfc_indications_0_deq );
//
// schedule portalIfc_indications_0_deq  CF ( portalIfc_messageSize_size,
// 					   portalIfc_indications_1_first,
// 					   portalIfc_indications_1_deq,
// 					   portalIfc_indications_1_notEmpty,
// 					   portalIfc_indications_2_first,
// 					   portalIfc_indications_2_deq,
// 					   portalIfc_indications_2_notEmpty,
// 					   ifc_startSwitching,
// 					   ifc_printSwStats,
// 					   ifc_start_shoal );
// schedule portalIfc_indications_0_deq  C ( portalIfc_indications_0_deq );
//
// schedule portalIfc_indications_0_notEmpty  CF ( portalIfc_messageSize_size,
// 						portalIfc_indications_0_first,
// 						portalIfc_indications_0_notEmpty,
// 						portalIfc_indications_1_first,
// 						portalIfc_indications_1_deq,
// 						portalIfc_indications_1_notEmpty,
// 						portalIfc_indications_2_first,
// 						portalIfc_indications_2_deq,
// 						portalIfc_indications_2_notEmpty,
// 						portalIfc_intr_status,
// 						portalIfc_intr_channel,
// 						ifc_printSwStats,
// 						ifc_start_shoal );
// schedule portalIfc_indications_0_notEmpty  SB ( portalIfc_indications_0_deq,
// 						ifc_startSwitching );
//
// schedule portalIfc_indications_1_first  CF ( portalIfc_messageSize_size,
// 					     portalIfc_indications_0_first,
// 					     portalIfc_indications_0_deq,
// 					     portalIfc_indications_0_notEmpty,
// 					     portalIfc_indications_1_first,
// 					     portalIfc_indications_1_notEmpty,
// 					     portalIfc_indications_2_first,
// 					     portalIfc_indications_2_deq,
// 					     portalIfc_indications_2_notEmpty,
// 					     portalIfc_intr_status,
// 					     portalIfc_intr_channel,
// 					     ifc_startSwitching,
// 					     ifc_printSwStats,
// 					     ifc_start_shoal );
// schedule portalIfc_indications_1_first  SB ( portalIfc_indications_1_deq );
//
// schedule portalIfc_indications_1_deq  CF ( portalIfc_messageSize_size,
// 					   portalIfc_indications_0_first,
// 					   portalIfc_indications_0_deq,
// 					   portalIfc_indications_0_notEmpty,
// 					   portalIfc_indications_2_first,
// 					   portalIfc_indications_2_deq,
// 					   portalIfc_indications_2_notEmpty,
// 					   ifc_startSwitching,
// 					   ifc_printSwStats,
// 					   ifc_start_shoal );
// schedule portalIfc_indications_1_deq  C ( portalIfc_indications_1_deq );
//
// schedule portalIfc_indications_1_notEmpty  CF ( portalIfc_messageSize_size,
// 						portalIfc_indications_0_first,
// 						portalIfc_indications_0_deq,
// 						portalIfc_indications_0_notEmpty,
// 						portalIfc_indications_1_first,
// 						portalIfc_indications_1_notEmpty,
// 						portalIfc_indications_2_first,
// 						portalIfc_indications_2_deq,
// 						portalIfc_indications_2_notEmpty,
// 						portalIfc_intr_status,
// 						portalIfc_intr_channel,
// 						ifc_startSwitching,
// 						ifc_start_shoal );
// schedule portalIfc_indications_1_notEmpty  SB ( portalIfc_indications_1_deq,
// 						ifc_printSwStats );
//
// schedule portalIfc_indications_2_first  CF ( portalIfc_messageSize_size,
// 					     portalIfc_indications_0_first,
// 					     portalIfc_indications_0_deq,
// 					     portalIfc_indications_0_notEmpty,
// 					     portalIfc_indications_1_first,
// 					     portalIfc_indications_1_deq,
// 					     portalIfc_indications_1_notEmpty,
// 					     portalIfc_indications_2_first,
// 					     portalIfc_indications_2_notEmpty,
// 					     portalIfc_intr_status,
// 					     portalIfc_intr_channel,
// 					     ifc_startSwitching,
// 					     ifc_printSwStats,
// 					     ifc_start_shoal );
// schedule portalIfc_indications_2_first  SB ( portalIfc_indications_2_deq );
//
// schedule portalIfc_indications_2_deq  CF ( portalIfc_messageSize_size,
// 					   portalIfc_indications_0_first,
// 					   portalIfc_indications_0_deq,
// 					   portalIfc_indications_0_notEmpty,
// 					   portalIfc_indications_1_first,
// 					   portalIfc_indications_1_deq,
// 					   portalIfc_indications_1_notEmpty,
// 					   ifc_startSwitching,
// 					   ifc_printSwStats,
// 					   ifc_start_shoal );
// schedule portalIfc_indications_2_deq  C ( portalIfc_indications_2_deq );
//
// schedule portalIfc_indications_2_notEmpty  CF ( portalIfc_messageSize_size,
// 						portalIfc_indications_0_first,
// 						portalIfc_indications_0_deq,
// 						portalIfc_indications_0_notEmpty,
// 						portalIfc_indications_1_first,
// 						portalIfc_indications_1_deq,
// 						portalIfc_indications_1_notEmpty,
// 						portalIfc_indications_2_first,
// 						portalIfc_indications_2_notEmpty,
// 						portalIfc_intr_status,
// 						portalIfc_intr_channel,
// 						ifc_startSwitching,
// 						ifc_printSwStats );
// schedule portalIfc_indications_2_notEmpty  SB ( portalIfc_indications_2_deq,
// 						ifc_start_shoal );
//
// schedule portalIfc_intr_status  CF ( portalIfc_messageSize_size,
// 				     portalIfc_indications_0_first,
// 				     portalIfc_indications_0_notEmpty,
// 				     portalIfc_indications_1_first,
// 				     portalIfc_indications_1_notEmpty,
// 				     portalIfc_indications_2_first,
// 				     portalIfc_indications_2_notEmpty,
// 				     portalIfc_intr_status,
// 				     portalIfc_intr_channel );
// schedule portalIfc_intr_status  SB ( portalIfc_indications_0_deq,
// 				     portalIfc_indications_1_deq,
// 				     portalIfc_indications_2_deq,
// 				     ifc_startSwitching,
// 				     ifc_printSwStats,
// 				     ifc_start_shoal );
//
// schedule portalIfc_intr_channel  CF ( portalIfc_messageSize_size,
// 				      portalIfc_indications_0_first,
// 				      portalIfc_indications_0_notEmpty,
// 				      portalIfc_indications_1_first,
// 				      portalIfc_indications_1_notEmpty,
// 				      portalIfc_indications_2_first,
// 				      portalIfc_indications_2_notEmpty,
// 				      portalIfc_intr_status,
// 				      portalIfc_intr_channel );
// schedule portalIfc_intr_channel  SB ( portalIfc_indications_0_deq,
// 				      portalIfc_indications_1_deq,
// 				      portalIfc_indications_2_deq,
// 				      ifc_startSwitching,
// 				      ifc_printSwStats,
// 				      ifc_start_shoal );
//
// schedule ifc_startSwitching  CF ( portalIfc_messageSize_size,
// 				  portalIfc_indications_0_first,
// 				  portalIfc_indications_0_deq,
// 				  portalIfc_indications_1_first,
// 				  portalIfc_indications_1_deq,
// 				  portalIfc_indications_1_notEmpty,
// 				  portalIfc_indications_2_first,
// 				  portalIfc_indications_2_deq,
// 				  portalIfc_indications_2_notEmpty,
// 				  ifc_printSwStats,
// 				  ifc_start_shoal );
// schedule ifc_startSwitching  C ( ifc_startSwitching );
//
// schedule ifc_printSwStats  CF ( portalIfc_messageSize_size,
// 				portalIfc_indications_0_first,
// 				portalIfc_indications_0_deq,
// 				portalIfc_indications_0_notEmpty,
// 				portalIfc_indications_1_first,
// 				portalIfc_indications_1_deq,
// 				portalIfc_indications_2_first,
// 				portalIfc_indications_2_deq,
// 				portalIfc_indications_2_notEmpty,
// 				ifc_startSwitching,
// 				ifc_start_shoal );
// schedule ifc_printSwStats  C ( ifc_printSwStats );
//
// schedule ifc_start_shoal  CF ( portalIfc_messageSize_size,
// 			       portalIfc_indications_0_first,
// 			       portalIfc_indications_0_deq,
// 			       portalIfc_indications_0_notEmpty,
// 			       portalIfc_indications_1_first,
// 			       portalIfc_indications_1_deq,
// 			       portalIfc_indications_1_notEmpty,
// 			       portalIfc_indications_2_first,
// 			       portalIfc_indications_2_deq,
// 			       ifc_startSwitching,
// 			       ifc_printSwStats );
// schedule ifc_start_shoal  C ( ifc_start_shoal );
//
//
// Ports:
// Name                         I/O  size props
// portalIfc_messageSize_size     O    16
// RDY_portalIfc_messageSize_size  O     1 const
// portalIfc_indications_0_first  O    32 reg
// RDY_portalIfc_indications_0_first  O     1 reg
// RDY_portalIfc_indications_0_deq  O     1 reg
// portalIfc_indications_0_notEmpty  O     1 reg
// RDY_portalIfc_indications_0_notEmpty  O     1 const
// portalIfc_indications_1_first  O    32 reg
// RDY_portalIfc_indications_1_first  O     1 reg
// RDY_portalIfc_indications_1_deq  O     1 reg
// portalIfc_indications_1_notEmpty  O     1 reg
// RDY_portalIfc_indications_1_notEmpty  O     1 const
// portalIfc_indications_2_first  O    32 reg
// RDY_portalIfc_indications_2_first  O     1 reg
// RDY_portalIfc_indications_2_deq  O     1 reg
// portalIfc_indications_2_notEmpty  O     1 reg
// RDY_portalIfc_indications_2_notEmpty  O     1 const
// portalIfc_intr_status          O     1
// RDY_portalIfc_intr_status      O     1 const
// portalIfc_intr_channel         O    32
// RDY_portalIfc_intr_channel     O     1 const
// RDY_ifc_startSwitching         O     1
// RDY_ifc_printSwStats           O     1
// RDY_ifc_start_shoal            O     1
// CLK                            I     1 clock
// RST_N                          I     1 reset
// portalIfc_messageSize_size_methodNumber  I    16
// ifc_startSwitching_reconfig_flag  I     8
// ifc_startSwitching_timeslot    I    64
// ifc_start_shoal_idx            I    32
// ifc_start_shoal_rate           I    16
// ifc_start_shoal_timeslot       I     8
// ifc_start_shoal_cycles         I    64
// EN_portalIfc_indications_0_deq  I     1
// EN_portalIfc_indications_1_deq  I     1
// EN_portalIfc_indications_2_deq  I     1
// EN_ifc_startSwitching          I     1
// EN_ifc_printSwStats            I     1
// EN_ifc_start_shoal             I     1
//
// Combinational paths from inputs to outputs:
//   portalIfc_messageSize_size_methodNumber -> portalIfc_messageSize_size
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

module mkShoalMultiSimTopRequestOutput(CLK,
				       RST_N,

				       portalIfc_messageSize_size_methodNumber,
				       portalIfc_messageSize_size,
				       RDY_portalIfc_messageSize_size,

				       portalIfc_indications_0_first,
				       RDY_portalIfc_indications_0_first,

				       EN_portalIfc_indications_0_deq,
				       RDY_portalIfc_indications_0_deq,

				       portalIfc_indications_0_notEmpty,
				       RDY_portalIfc_indications_0_notEmpty,

				       portalIfc_indications_1_first,
				       RDY_portalIfc_indications_1_first,

				       EN_portalIfc_indications_1_deq,
				       RDY_portalIfc_indications_1_deq,

				       portalIfc_indications_1_notEmpty,
				       RDY_portalIfc_indications_1_notEmpty,

				       portalIfc_indications_2_first,
				       RDY_portalIfc_indications_2_first,

				       EN_portalIfc_indications_2_deq,
				       RDY_portalIfc_indications_2_deq,

				       portalIfc_indications_2_notEmpty,
				       RDY_portalIfc_indications_2_notEmpty,

				       portalIfc_intr_status,
				       RDY_portalIfc_intr_status,

				       portalIfc_intr_channel,
				       RDY_portalIfc_intr_channel,

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
				       RDY_ifc_start_shoal);
  input  CLK;
  input  RST_N;

  // value method portalIfc_messageSize_size
  input  [15 : 0] portalIfc_messageSize_size_methodNumber;
  output [15 : 0] portalIfc_messageSize_size;
  output RDY_portalIfc_messageSize_size;

  // value method portalIfc_indications_0_first
  output [31 : 0] portalIfc_indications_0_first;
  output RDY_portalIfc_indications_0_first;

  // action method portalIfc_indications_0_deq
  input  EN_portalIfc_indications_0_deq;
  output RDY_portalIfc_indications_0_deq;

  // value method portalIfc_indications_0_notEmpty
  output portalIfc_indications_0_notEmpty;
  output RDY_portalIfc_indications_0_notEmpty;

  // value method portalIfc_indications_1_first
  output [31 : 0] portalIfc_indications_1_first;
  output RDY_portalIfc_indications_1_first;

  // action method portalIfc_indications_1_deq
  input  EN_portalIfc_indications_1_deq;
  output RDY_portalIfc_indications_1_deq;

  // value method portalIfc_indications_1_notEmpty
  output portalIfc_indications_1_notEmpty;
  output RDY_portalIfc_indications_1_notEmpty;

  // value method portalIfc_indications_2_first
  output [31 : 0] portalIfc_indications_2_first;
  output RDY_portalIfc_indications_2_first;

  // action method portalIfc_indications_2_deq
  input  EN_portalIfc_indications_2_deq;
  output RDY_portalIfc_indications_2_deq;

  // value method portalIfc_indications_2_notEmpty
  output portalIfc_indications_2_notEmpty;
  output RDY_portalIfc_indications_2_notEmpty;

  // value method portalIfc_intr_status
  output portalIfc_intr_status;
  output RDY_portalIfc_intr_status;

  // value method portalIfc_intr_channel
  output [31 : 0] portalIfc_intr_channel;
  output RDY_portalIfc_intr_channel;

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

  // signals for module outputs
  wire [31 : 0] portalIfc_indications_0_first,
		portalIfc_indications_1_first,
		portalIfc_indications_2_first,
		portalIfc_intr_channel;
  wire [15 : 0] portalIfc_messageSize_size;
  wire RDY_ifc_printSwStats,
       RDY_ifc_startSwitching,
       RDY_ifc_start_shoal,
       RDY_portalIfc_indications_0_deq,
       RDY_portalIfc_indications_0_first,
       RDY_portalIfc_indications_0_notEmpty,
       RDY_portalIfc_indications_1_deq,
       RDY_portalIfc_indications_1_first,
       RDY_portalIfc_indications_1_notEmpty,
       RDY_portalIfc_indications_2_deq,
       RDY_portalIfc_indications_2_first,
       RDY_portalIfc_indications_2_notEmpty,
       RDY_portalIfc_intr_channel,
       RDY_portalIfc_intr_status,
       RDY_portalIfc_messageSize_size,
       portalIfc_indications_0_notEmpty,
       portalIfc_indications_1_notEmpty,
       portalIfc_indications_2_notEmpty,
       portalIfc_intr_status;

  // ports of submodule indicationPipes
  wire [119 : 0] indicationPipes_methods_start_shoal_enq_v;
  wire [71 : 0] indicationPipes_methods_startSwitching_enq_v;
  wire [31 : 0] indicationPipes_methods_printSwStats_enq_v,
		indicationPipes_portalIfc_indications_0_first,
		indicationPipes_portalIfc_indications_1_first,
		indicationPipes_portalIfc_indications_2_first,
		indicationPipes_portalIfc_intr_channel;
  wire [15 : 0] indicationPipes_portalIfc_messageSize_size,
		indicationPipes_portalIfc_messageSize_size_methodNumber;
  wire indicationPipes_EN_methods_printSwStats_enq,
       indicationPipes_EN_methods_startSwitching_enq,
       indicationPipes_EN_methods_start_shoal_enq,
       indicationPipes_EN_portalIfc_indications_0_deq,
       indicationPipes_EN_portalIfc_indications_1_deq,
       indicationPipes_EN_portalIfc_indications_2_deq,
       indicationPipes_RDY_methods_printSwStats_enq,
       indicationPipes_RDY_methods_startSwitching_enq,
       indicationPipes_RDY_methods_start_shoal_enq,
       indicationPipes_RDY_portalIfc_indications_0_deq,
       indicationPipes_RDY_portalIfc_indications_0_first,
       indicationPipes_RDY_portalIfc_indications_1_deq,
       indicationPipes_RDY_portalIfc_indications_1_first,
       indicationPipes_RDY_portalIfc_indications_2_deq,
       indicationPipes_RDY_portalIfc_indications_2_first,
       indicationPipes_portalIfc_indications_0_notEmpty,
       indicationPipes_portalIfc_indications_1_notEmpty,
       indicationPipes_portalIfc_indications_2_notEmpty,
       indicationPipes_portalIfc_intr_status;

  // value method portalIfc_messageSize_size
  assign portalIfc_messageSize_size =
	     indicationPipes_portalIfc_messageSize_size ;
  assign RDY_portalIfc_messageSize_size = 1'd1 ;

  // value method portalIfc_indications_0_first
  assign portalIfc_indications_0_first =
	     indicationPipes_portalIfc_indications_0_first ;
  assign RDY_portalIfc_indications_0_first =
	     indicationPipes_RDY_portalIfc_indications_0_first ;

  // action method portalIfc_indications_0_deq
  assign RDY_portalIfc_indications_0_deq =
	     indicationPipes_RDY_portalIfc_indications_0_deq ;

  // value method portalIfc_indications_0_notEmpty
  assign portalIfc_indications_0_notEmpty =
	     indicationPipes_portalIfc_indications_0_notEmpty ;
  assign RDY_portalIfc_indications_0_notEmpty = 1'd1 ;

  // value method portalIfc_indications_1_first
  assign portalIfc_indications_1_first =
	     indicationPipes_portalIfc_indications_1_first ;
  assign RDY_portalIfc_indications_1_first =
	     indicationPipes_RDY_portalIfc_indications_1_first ;

  // action method portalIfc_indications_1_deq
  assign RDY_portalIfc_indications_1_deq =
	     indicationPipes_RDY_portalIfc_indications_1_deq ;

  // value method portalIfc_indications_1_notEmpty
  assign portalIfc_indications_1_notEmpty =
	     indicationPipes_portalIfc_indications_1_notEmpty ;
  assign RDY_portalIfc_indications_1_notEmpty = 1'd1 ;

  // value method portalIfc_indications_2_first
  assign portalIfc_indications_2_first =
	     indicationPipes_portalIfc_indications_2_first ;
  assign RDY_portalIfc_indications_2_first =
	     indicationPipes_RDY_portalIfc_indications_2_first ;

  // action method portalIfc_indications_2_deq
  assign RDY_portalIfc_indications_2_deq =
	     indicationPipes_RDY_portalIfc_indications_2_deq ;

  // value method portalIfc_indications_2_notEmpty
  assign portalIfc_indications_2_notEmpty =
	     indicationPipes_portalIfc_indications_2_notEmpty ;
  assign RDY_portalIfc_indications_2_notEmpty = 1'd1 ;

  // value method portalIfc_intr_status
  assign portalIfc_intr_status = indicationPipes_portalIfc_intr_status ;
  assign RDY_portalIfc_intr_status = 1'd1 ;

  // value method portalIfc_intr_channel
  assign portalIfc_intr_channel = indicationPipes_portalIfc_intr_channel ;
  assign RDY_portalIfc_intr_channel = 1'd1 ;

  // action method ifc_startSwitching
  assign RDY_ifc_startSwitching =
	     indicationPipes_RDY_methods_startSwitching_enq ;

  // action method ifc_printSwStats
  assign RDY_ifc_printSwStats = indicationPipes_RDY_methods_printSwStats_enq ;

  // action method ifc_start_shoal
  assign RDY_ifc_start_shoal = indicationPipes_RDY_methods_start_shoal_enq ;

  // submodule indicationPipes
  mkShoalMultiSimTopRequestOutputPipes indicationPipes(.CLK(CLK),
						       .RST_N(RST_N),
						       .methods_printSwStats_enq_v(indicationPipes_methods_printSwStats_enq_v),
						       .methods_startSwitching_enq_v(indicationPipes_methods_startSwitching_enq_v),
						       .methods_start_shoal_enq_v(indicationPipes_methods_start_shoal_enq_v),
						       .portalIfc_messageSize_size_methodNumber(indicationPipes_portalIfc_messageSize_size_methodNumber),
						       .EN_methods_startSwitching_enq(indicationPipes_EN_methods_startSwitching_enq),
						       .EN_methods_printSwStats_enq(indicationPipes_EN_methods_printSwStats_enq),
						       .EN_methods_start_shoal_enq(indicationPipes_EN_methods_start_shoal_enq),
						       .EN_portalIfc_indications_0_deq(indicationPipes_EN_portalIfc_indications_0_deq),
						       .EN_portalIfc_indications_1_deq(indicationPipes_EN_portalIfc_indications_1_deq),
						       .EN_portalIfc_indications_2_deq(indicationPipes_EN_portalIfc_indications_2_deq),
						       .RDY_methods_startSwitching_enq(indicationPipes_RDY_methods_startSwitching_enq),
						       .methods_startSwitching_notFull(),
						       .RDY_methods_startSwitching_notFull(),
						       .RDY_methods_printSwStats_enq(indicationPipes_RDY_methods_printSwStats_enq),
						       .methods_printSwStats_notFull(),
						       .RDY_methods_printSwStats_notFull(),
						       .RDY_methods_start_shoal_enq(indicationPipes_RDY_methods_start_shoal_enq),
						       .methods_start_shoal_notFull(),
						       .RDY_methods_start_shoal_notFull(),
						       .portalIfc_messageSize_size(indicationPipes_portalIfc_messageSize_size),
						       .RDY_portalIfc_messageSize_size(),
						       .portalIfc_indications_0_first(indicationPipes_portalIfc_indications_0_first),
						       .RDY_portalIfc_indications_0_first(indicationPipes_RDY_portalIfc_indications_0_first),
						       .RDY_portalIfc_indications_0_deq(indicationPipes_RDY_portalIfc_indications_0_deq),
						       .portalIfc_indications_0_notEmpty(indicationPipes_portalIfc_indications_0_notEmpty),
						       .RDY_portalIfc_indications_0_notEmpty(),
						       .portalIfc_indications_1_first(indicationPipes_portalIfc_indications_1_first),
						       .RDY_portalIfc_indications_1_first(indicationPipes_RDY_portalIfc_indications_1_first),
						       .RDY_portalIfc_indications_1_deq(indicationPipes_RDY_portalIfc_indications_1_deq),
						       .portalIfc_indications_1_notEmpty(indicationPipes_portalIfc_indications_1_notEmpty),
						       .RDY_portalIfc_indications_1_notEmpty(),
						       .portalIfc_indications_2_first(indicationPipes_portalIfc_indications_2_first),
						       .RDY_portalIfc_indications_2_first(indicationPipes_RDY_portalIfc_indications_2_first),
						       .RDY_portalIfc_indications_2_deq(indicationPipes_RDY_portalIfc_indications_2_deq),
						       .portalIfc_indications_2_notEmpty(indicationPipes_portalIfc_indications_2_notEmpty),
						       .RDY_portalIfc_indications_2_notEmpty(),
						       .portalIfc_intr_status(indicationPipes_portalIfc_intr_status),
						       .RDY_portalIfc_intr_status(),
						       .portalIfc_intr_channel(indicationPipes_portalIfc_intr_channel),
						       .RDY_portalIfc_intr_channel());

  // submodule indicationPipes
  assign indicationPipes_methods_printSwStats_enq_v = 32'd0 ;
  assign indicationPipes_methods_startSwitching_enq_v =
	     { ifc_startSwitching_reconfig_flag,
	       ifc_startSwitching_timeslot } ;
  assign indicationPipes_methods_start_shoal_enq_v =
	     { ifc_start_shoal_idx,
	       ifc_start_shoal_rate,
	       ifc_start_shoal_timeslot,
	       ifc_start_shoal_cycles } ;
  assign indicationPipes_portalIfc_messageSize_size_methodNumber =
	     portalIfc_messageSize_size_methodNumber ;
  assign indicationPipes_EN_methods_startSwitching_enq =
	     EN_ifc_startSwitching ;
  assign indicationPipes_EN_methods_printSwStats_enq = EN_ifc_printSwStats ;
  assign indicationPipes_EN_methods_start_shoal_enq = EN_ifc_start_shoal ;
  assign indicationPipes_EN_portalIfc_indications_0_deq =
	     EN_portalIfc_indications_0_deq ;
  assign indicationPipes_EN_portalIfc_indications_1_deq =
	     EN_portalIfc_indications_1_deq ;
  assign indicationPipes_EN_portalIfc_indications_2_deq =
	     EN_portalIfc_indications_2_deq ;
endmodule  // mkShoalMultiSimTopRequestOutput

