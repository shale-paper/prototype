//
// Generated by Bluespec Compiler, version 2023.01 (build 52adafa5)
//
// timestamp removed
//
// BVI format method schedule info:
// schedule portalIfc_messageSize_size  CF ( portalIfc_messageSize_size,
// 					  portalIfc_requests_0_enq,
// 					  portalIfc_requests_0_notFull,
// 					  portalIfc_requests_1_enq,
// 					  portalIfc_requests_1_notFull,
// 					  portalIfc_requests_2_enq,
// 					  portalIfc_requests_2_notFull,
// 					  portalIfc_intr_status,
// 					  portalIfc_intr_channel,
// 					  pipes_startSwitching_PipeOut_first,
// 					  pipes_startSwitching_PipeOut_deq,
// 					  pipes_startSwitching_PipeOut_notEmpty,
// 					  pipes_printSwStats_PipeOut_first,
// 					  pipes_printSwStats_PipeOut_deq,
// 					  pipes_printSwStats_PipeOut_notEmpty,
// 					  pipes_start_shoal_PipeOut_first,
// 					  pipes_start_shoal_PipeOut_deq,
// 					  pipes_start_shoal_PipeOut_notEmpty );
//
// schedule portalIfc_requests_0_enq  CF ( portalIfc_messageSize_size,
// 					portalIfc_requests_1_enq,
// 					portalIfc_requests_1_notFull,
// 					portalIfc_requests_2_enq,
// 					portalIfc_requests_2_notFull,
// 					portalIfc_intr_status,
// 					portalIfc_intr_channel,
// 					pipes_startSwitching_PipeOut_first,
// 					pipes_printSwStats_PipeOut_first,
// 					pipes_printSwStats_PipeOut_deq,
// 					pipes_printSwStats_PipeOut_notEmpty,
// 					pipes_start_shoal_PipeOut_first,
// 					pipes_start_shoal_PipeOut_deq,
// 					pipes_start_shoal_PipeOut_notEmpty );
// schedule portalIfc_requests_0_enq  SB ( pipes_startSwitching_PipeOut_deq );
// schedule portalIfc_requests_0_enq  C ( portalIfc_requests_0_enq );
//
// schedule portalIfc_requests_0_notFull  CF ( portalIfc_messageSize_size,
// 					    portalIfc_requests_0_notFull,
// 					    portalIfc_requests_1_enq,
// 					    portalIfc_requests_1_notFull,
// 					    portalIfc_requests_2_enq,
// 					    portalIfc_requests_2_notFull,
// 					    portalIfc_intr_status,
// 					    portalIfc_intr_channel,
// 					    pipes_startSwitching_PipeOut_first,
// 					    pipes_startSwitching_PipeOut_notEmpty,
// 					    pipes_printSwStats_PipeOut_first,
// 					    pipes_printSwStats_PipeOut_deq,
// 					    pipes_printSwStats_PipeOut_notEmpty,
// 					    pipes_start_shoal_PipeOut_first,
// 					    pipes_start_shoal_PipeOut_deq,
// 					    pipes_start_shoal_PipeOut_notEmpty );
// schedule portalIfc_requests_0_notFull  SB ( portalIfc_requests_0_enq,
// 					    pipes_startSwitching_PipeOut_deq );
//
// schedule portalIfc_requests_1_enq  CF ( portalIfc_messageSize_size,
// 					portalIfc_requests_0_enq,
// 					portalIfc_requests_0_notFull,
// 					portalIfc_requests_2_enq,
// 					portalIfc_requests_2_notFull,
// 					portalIfc_intr_status,
// 					portalIfc_intr_channel,
// 					pipes_startSwitching_PipeOut_first,
// 					pipes_startSwitching_PipeOut_deq,
// 					pipes_startSwitching_PipeOut_notEmpty,
// 					pipes_printSwStats_PipeOut_first,
// 					pipes_start_shoal_PipeOut_first,
// 					pipes_start_shoal_PipeOut_deq,
// 					pipes_start_shoal_PipeOut_notEmpty );
// schedule portalIfc_requests_1_enq  SB ( pipes_printSwStats_PipeOut_deq );
// schedule portalIfc_requests_1_enq  C ( portalIfc_requests_1_enq );
//
// schedule portalIfc_requests_1_notFull  CF ( portalIfc_messageSize_size,
// 					    portalIfc_requests_0_enq,
// 					    portalIfc_requests_0_notFull,
// 					    portalIfc_requests_1_notFull,
// 					    portalIfc_requests_2_enq,
// 					    portalIfc_requests_2_notFull,
// 					    portalIfc_intr_status,
// 					    portalIfc_intr_channel,
// 					    pipes_startSwitching_PipeOut_first,
// 					    pipes_startSwitching_PipeOut_deq,
// 					    pipes_startSwitching_PipeOut_notEmpty,
// 					    pipes_printSwStats_PipeOut_first,
// 					    pipes_printSwStats_PipeOut_notEmpty,
// 					    pipes_start_shoal_PipeOut_first,
// 					    pipes_start_shoal_PipeOut_deq,
// 					    pipes_start_shoal_PipeOut_notEmpty );
// schedule portalIfc_requests_1_notFull  SB ( portalIfc_requests_1_enq,
// 					    pipes_printSwStats_PipeOut_deq );
//
// schedule portalIfc_requests_2_enq  CF ( portalIfc_messageSize_size,
// 					portalIfc_requests_0_enq,
// 					portalIfc_requests_0_notFull,
// 					portalIfc_requests_1_enq,
// 					portalIfc_requests_1_notFull,
// 					portalIfc_intr_status,
// 					portalIfc_intr_channel,
// 					pipes_startSwitching_PipeOut_first,
// 					pipes_startSwitching_PipeOut_deq,
// 					pipes_startSwitching_PipeOut_notEmpty,
// 					pipes_printSwStats_PipeOut_first,
// 					pipes_printSwStats_PipeOut_deq,
// 					pipes_printSwStats_PipeOut_notEmpty,
// 					pipes_start_shoal_PipeOut_first );
// schedule portalIfc_requests_2_enq  SB ( pipes_start_shoal_PipeOut_deq );
// schedule portalIfc_requests_2_enq  C ( portalIfc_requests_2_enq );
//
// schedule portalIfc_requests_2_notFull  CF ( portalIfc_messageSize_size,
// 					    portalIfc_requests_0_enq,
// 					    portalIfc_requests_0_notFull,
// 					    portalIfc_requests_1_enq,
// 					    portalIfc_requests_1_notFull,
// 					    portalIfc_requests_2_notFull,
// 					    portalIfc_intr_status,
// 					    portalIfc_intr_channel,
// 					    pipes_startSwitching_PipeOut_first,
// 					    pipes_startSwitching_PipeOut_deq,
// 					    pipes_startSwitching_PipeOut_notEmpty,
// 					    pipes_printSwStats_PipeOut_first,
// 					    pipes_printSwStats_PipeOut_deq,
// 					    pipes_printSwStats_PipeOut_notEmpty,
// 					    pipes_start_shoal_PipeOut_first,
// 					    pipes_start_shoal_PipeOut_notEmpty );
// schedule portalIfc_requests_2_notFull  SB ( portalIfc_requests_2_enq,
// 					    pipes_start_shoal_PipeOut_deq );
//
// schedule portalIfc_intr_status  CF ( portalIfc_messageSize_size,
// 				     portalIfc_requests_0_enq,
// 				     portalIfc_requests_0_notFull,
// 				     portalIfc_requests_1_enq,
// 				     portalIfc_requests_1_notFull,
// 				     portalIfc_requests_2_enq,
// 				     portalIfc_requests_2_notFull,
// 				     portalIfc_intr_status,
// 				     portalIfc_intr_channel,
// 				     pipes_startSwitching_PipeOut_first,
// 				     pipes_startSwitching_PipeOut_deq,
// 				     pipes_startSwitching_PipeOut_notEmpty,
// 				     pipes_printSwStats_PipeOut_first,
// 				     pipes_printSwStats_PipeOut_deq,
// 				     pipes_printSwStats_PipeOut_notEmpty,
// 				     pipes_start_shoal_PipeOut_first,
// 				     pipes_start_shoal_PipeOut_deq,
// 				     pipes_start_shoal_PipeOut_notEmpty );
//
// schedule portalIfc_intr_channel  CF ( portalIfc_messageSize_size,
// 				      portalIfc_requests_0_enq,
// 				      portalIfc_requests_0_notFull,
// 				      portalIfc_requests_1_enq,
// 				      portalIfc_requests_1_notFull,
// 				      portalIfc_requests_2_enq,
// 				      portalIfc_requests_2_notFull,
// 				      portalIfc_intr_status,
// 				      portalIfc_intr_channel,
// 				      pipes_startSwitching_PipeOut_first,
// 				      pipes_startSwitching_PipeOut_deq,
// 				      pipes_startSwitching_PipeOut_notEmpty,
// 				      pipes_printSwStats_PipeOut_first,
// 				      pipes_printSwStats_PipeOut_deq,
// 				      pipes_printSwStats_PipeOut_notEmpty,
// 				      pipes_start_shoal_PipeOut_first,
// 				      pipes_start_shoal_PipeOut_deq,
// 				      pipes_start_shoal_PipeOut_notEmpty );
//
// schedule pipes_startSwitching_PipeOut_first  CF ( portalIfc_messageSize_size,
// 						  portalIfc_requests_0_enq,
// 						  portalIfc_requests_0_notFull,
// 						  portalIfc_requests_1_enq,
// 						  portalIfc_requests_1_notFull,
// 						  portalIfc_requests_2_enq,
// 						  portalIfc_requests_2_notFull,
// 						  portalIfc_intr_status,
// 						  portalIfc_intr_channel,
// 						  pipes_startSwitching_PipeOut_first,
// 						  pipes_startSwitching_PipeOut_notEmpty,
// 						  pipes_printSwStats_PipeOut_first,
// 						  pipes_printSwStats_PipeOut_deq,
// 						  pipes_printSwStats_PipeOut_notEmpty,
// 						  pipes_start_shoal_PipeOut_first,
// 						  pipes_start_shoal_PipeOut_deq,
// 						  pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_startSwitching_PipeOut_first  SB ( pipes_startSwitching_PipeOut_deq );
//
// schedule pipes_startSwitching_PipeOut_deq  CF ( portalIfc_messageSize_size,
// 						portalIfc_requests_1_enq,
// 						portalIfc_requests_1_notFull,
// 						portalIfc_requests_2_enq,
// 						portalIfc_requests_2_notFull,
// 						portalIfc_intr_status,
// 						portalIfc_intr_channel,
// 						pipes_printSwStats_PipeOut_first,
// 						pipes_printSwStats_PipeOut_deq,
// 						pipes_printSwStats_PipeOut_notEmpty,
// 						pipes_start_shoal_PipeOut_first,
// 						pipes_start_shoal_PipeOut_deq,
// 						pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_startSwitching_PipeOut_deq  C ( pipes_startSwitching_PipeOut_deq );
//
// schedule pipes_startSwitching_PipeOut_notEmpty  CF ( portalIfc_messageSize_size,
// 						     portalIfc_requests_0_notFull,
// 						     portalIfc_requests_1_enq,
// 						     portalIfc_requests_1_notFull,
// 						     portalIfc_requests_2_enq,
// 						     portalIfc_requests_2_notFull,
// 						     portalIfc_intr_status,
// 						     portalIfc_intr_channel,
// 						     pipes_startSwitching_PipeOut_first,
// 						     pipes_startSwitching_PipeOut_notEmpty,
// 						     pipes_printSwStats_PipeOut_first,
// 						     pipes_printSwStats_PipeOut_deq,
// 						     pipes_printSwStats_PipeOut_notEmpty,
// 						     pipes_start_shoal_PipeOut_first,
// 						     pipes_start_shoal_PipeOut_deq,
// 						     pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_startSwitching_PipeOut_notEmpty  SB ( portalIfc_requests_0_enq,
// 						     pipes_startSwitching_PipeOut_deq );
//
// schedule pipes_printSwStats_PipeOut_first  CF ( portalIfc_messageSize_size,
// 						portalIfc_requests_0_enq,
// 						portalIfc_requests_0_notFull,
// 						portalIfc_requests_1_enq,
// 						portalIfc_requests_1_notFull,
// 						portalIfc_requests_2_enq,
// 						portalIfc_requests_2_notFull,
// 						portalIfc_intr_status,
// 						portalIfc_intr_channel,
// 						pipes_startSwitching_PipeOut_first,
// 						pipes_startSwitching_PipeOut_deq,
// 						pipes_startSwitching_PipeOut_notEmpty,
// 						pipes_printSwStats_PipeOut_first,
// 						pipes_printSwStats_PipeOut_notEmpty,
// 						pipes_start_shoal_PipeOut_first,
// 						pipes_start_shoal_PipeOut_deq,
// 						pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_printSwStats_PipeOut_first  SB ( pipes_printSwStats_PipeOut_deq );
//
// schedule pipes_printSwStats_PipeOut_deq  CF ( portalIfc_messageSize_size,
// 					      portalIfc_requests_0_enq,
// 					      portalIfc_requests_0_notFull,
// 					      portalIfc_requests_2_enq,
// 					      portalIfc_requests_2_notFull,
// 					      portalIfc_intr_status,
// 					      portalIfc_intr_channel,
// 					      pipes_startSwitching_PipeOut_first,
// 					      pipes_startSwitching_PipeOut_deq,
// 					      pipes_startSwitching_PipeOut_notEmpty,
// 					      pipes_start_shoal_PipeOut_first,
// 					      pipes_start_shoal_PipeOut_deq,
// 					      pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_printSwStats_PipeOut_deq  C ( pipes_printSwStats_PipeOut_deq );
//
// schedule pipes_printSwStats_PipeOut_notEmpty  CF ( portalIfc_messageSize_size,
// 						   portalIfc_requests_0_enq,
// 						   portalIfc_requests_0_notFull,
// 						   portalIfc_requests_1_notFull,
// 						   portalIfc_requests_2_enq,
// 						   portalIfc_requests_2_notFull,
// 						   portalIfc_intr_status,
// 						   portalIfc_intr_channel,
// 						   pipes_startSwitching_PipeOut_first,
// 						   pipes_startSwitching_PipeOut_deq,
// 						   pipes_startSwitching_PipeOut_notEmpty,
// 						   pipes_printSwStats_PipeOut_first,
// 						   pipes_printSwStats_PipeOut_notEmpty,
// 						   pipes_start_shoal_PipeOut_first,
// 						   pipes_start_shoal_PipeOut_deq,
// 						   pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_printSwStats_PipeOut_notEmpty  SB ( portalIfc_requests_1_enq,
// 						   pipes_printSwStats_PipeOut_deq );
//
// schedule pipes_start_shoal_PipeOut_first  CF ( portalIfc_messageSize_size,
// 					       portalIfc_requests_0_enq,
// 					       portalIfc_requests_0_notFull,
// 					       portalIfc_requests_1_enq,
// 					       portalIfc_requests_1_notFull,
// 					       portalIfc_requests_2_enq,
// 					       portalIfc_requests_2_notFull,
// 					       portalIfc_intr_status,
// 					       portalIfc_intr_channel,
// 					       pipes_startSwitching_PipeOut_first,
// 					       pipes_startSwitching_PipeOut_deq,
// 					       pipes_startSwitching_PipeOut_notEmpty,
// 					       pipes_printSwStats_PipeOut_first,
// 					       pipes_printSwStats_PipeOut_deq,
// 					       pipes_printSwStats_PipeOut_notEmpty,
// 					       pipes_start_shoal_PipeOut_first,
// 					       pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_start_shoal_PipeOut_first  SB ( pipes_start_shoal_PipeOut_deq );
//
// schedule pipes_start_shoal_PipeOut_deq  CF ( portalIfc_messageSize_size,
// 					     portalIfc_requests_0_enq,
// 					     portalIfc_requests_0_notFull,
// 					     portalIfc_requests_1_enq,
// 					     portalIfc_requests_1_notFull,
// 					     portalIfc_intr_status,
// 					     portalIfc_intr_channel,
// 					     pipes_startSwitching_PipeOut_first,
// 					     pipes_startSwitching_PipeOut_deq,
// 					     pipes_startSwitching_PipeOut_notEmpty,
// 					     pipes_printSwStats_PipeOut_first,
// 					     pipes_printSwStats_PipeOut_deq,
// 					     pipes_printSwStats_PipeOut_notEmpty );
// schedule pipes_start_shoal_PipeOut_deq  C ( pipes_start_shoal_PipeOut_deq );
//
// schedule pipes_start_shoal_PipeOut_notEmpty  CF ( portalIfc_messageSize_size,
// 						  portalIfc_requests_0_enq,
// 						  portalIfc_requests_0_notFull,
// 						  portalIfc_requests_1_enq,
// 						  portalIfc_requests_1_notFull,
// 						  portalIfc_requests_2_notFull,
// 						  portalIfc_intr_status,
// 						  portalIfc_intr_channel,
// 						  pipes_startSwitching_PipeOut_first,
// 						  pipes_startSwitching_PipeOut_deq,
// 						  pipes_startSwitching_PipeOut_notEmpty,
// 						  pipes_printSwStats_PipeOut_first,
// 						  pipes_printSwStats_PipeOut_deq,
// 						  pipes_printSwStats_PipeOut_notEmpty,
// 						  pipes_start_shoal_PipeOut_first,
// 						  pipes_start_shoal_PipeOut_notEmpty );
// schedule pipes_start_shoal_PipeOut_notEmpty  SB ( portalIfc_requests_2_enq,
// 						  pipes_start_shoal_PipeOut_deq );
//
//
// Ports:
// Name                         I/O  size props
// portalIfc_messageSize_size     O    16
// RDY_portalIfc_messageSize_size  O     1 const
// RDY_portalIfc_requests_0_enq   O     1
// portalIfc_requests_0_notFull   O     1 reg
// RDY_portalIfc_requests_0_notFull  O     1 const
// RDY_portalIfc_requests_1_enq   O     1 reg
// portalIfc_requests_1_notFull   O     1 reg
// RDY_portalIfc_requests_1_notFull  O     1 const
// RDY_portalIfc_requests_2_enq   O     1
// portalIfc_requests_2_notFull   O     1 reg
// RDY_portalIfc_requests_2_notFull  O     1 const
// portalIfc_intr_status          O     1 const
// RDY_portalIfc_intr_status      O     1 const
// portalIfc_intr_channel         O    32 const
// RDY_portalIfc_intr_channel     O     1 const
// pipes_startSwitching_PipeOut_first  O    72 reg
// RDY_pipes_startSwitching_PipeOut_first  O     1 reg
// RDY_pipes_startSwitching_PipeOut_deq  O     1 reg
// pipes_startSwitching_PipeOut_notEmpty  O     1 reg
// RDY_pipes_startSwitching_PipeOut_notEmpty  O     1 const
// pipes_printSwStats_PipeOut_first  O    32 reg
// RDY_pipes_printSwStats_PipeOut_first  O     1 reg
// RDY_pipes_printSwStats_PipeOut_deq  O     1 reg
// pipes_printSwStats_PipeOut_notEmpty  O     1 reg
// RDY_pipes_printSwStats_PipeOut_notEmpty  O     1 const
// pipes_start_shoal_PipeOut_first  O   120 reg
// RDY_pipes_start_shoal_PipeOut_first  O     1 reg
// RDY_pipes_start_shoal_PipeOut_deq  O     1 reg
// pipes_start_shoal_PipeOut_notEmpty  O     1 reg
// RDY_pipes_start_shoal_PipeOut_notEmpty  O     1 const
// CLK                            I     1 clock
// RST_N                          I     1 reset
// portalIfc_messageSize_size_methodNumber  I    16
// portalIfc_requests_0_enq_v     I    32 reg
// portalIfc_requests_1_enq_v     I    32 reg
// portalIfc_requests_2_enq_v     I    32 reg
// EN_portalIfc_requests_0_enq    I     1
// EN_portalIfc_requests_1_enq    I     1
// EN_portalIfc_requests_2_enq    I     1
// EN_pipes_startSwitching_PipeOut_deq  I     1
// EN_pipes_printSwStats_PipeOut_deq  I     1
// EN_pipes_start_shoal_PipeOut_deq  I     1
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

module mkShoalMultiSimTopRequestInput(CLK,
				      RST_N,

				      portalIfc_messageSize_size_methodNumber,
				      portalIfc_messageSize_size,
				      RDY_portalIfc_messageSize_size,

				      portalIfc_requests_0_enq_v,
				      EN_portalIfc_requests_0_enq,
				      RDY_portalIfc_requests_0_enq,

				      portalIfc_requests_0_notFull,
				      RDY_portalIfc_requests_0_notFull,

				      portalIfc_requests_1_enq_v,
				      EN_portalIfc_requests_1_enq,
				      RDY_portalIfc_requests_1_enq,

				      portalIfc_requests_1_notFull,
				      RDY_portalIfc_requests_1_notFull,

				      portalIfc_requests_2_enq_v,
				      EN_portalIfc_requests_2_enq,
				      RDY_portalIfc_requests_2_enq,

				      portalIfc_requests_2_notFull,
				      RDY_portalIfc_requests_2_notFull,

				      portalIfc_intr_status,
				      RDY_portalIfc_intr_status,

				      portalIfc_intr_channel,
				      RDY_portalIfc_intr_channel,

				      pipes_startSwitching_PipeOut_first,
				      RDY_pipes_startSwitching_PipeOut_first,

				      EN_pipes_startSwitching_PipeOut_deq,
				      RDY_pipes_startSwitching_PipeOut_deq,

				      pipes_startSwitching_PipeOut_notEmpty,
				      RDY_pipes_startSwitching_PipeOut_notEmpty,

				      pipes_printSwStats_PipeOut_first,
				      RDY_pipes_printSwStats_PipeOut_first,

				      EN_pipes_printSwStats_PipeOut_deq,
				      RDY_pipes_printSwStats_PipeOut_deq,

				      pipes_printSwStats_PipeOut_notEmpty,
				      RDY_pipes_printSwStats_PipeOut_notEmpty,

				      pipes_start_shoal_PipeOut_first,
				      RDY_pipes_start_shoal_PipeOut_first,

				      EN_pipes_start_shoal_PipeOut_deq,
				      RDY_pipes_start_shoal_PipeOut_deq,

				      pipes_start_shoal_PipeOut_notEmpty,
				      RDY_pipes_start_shoal_PipeOut_notEmpty);
  input  CLK;
  input  RST_N;

  // value method portalIfc_messageSize_size
  input  [15 : 0] portalIfc_messageSize_size_methodNumber;
  output [15 : 0] portalIfc_messageSize_size;
  output RDY_portalIfc_messageSize_size;

  // action method portalIfc_requests_0_enq
  input  [31 : 0] portalIfc_requests_0_enq_v;
  input  EN_portalIfc_requests_0_enq;
  output RDY_portalIfc_requests_0_enq;

  // value method portalIfc_requests_0_notFull
  output portalIfc_requests_0_notFull;
  output RDY_portalIfc_requests_0_notFull;

  // action method portalIfc_requests_1_enq
  input  [31 : 0] portalIfc_requests_1_enq_v;
  input  EN_portalIfc_requests_1_enq;
  output RDY_portalIfc_requests_1_enq;

  // value method portalIfc_requests_1_notFull
  output portalIfc_requests_1_notFull;
  output RDY_portalIfc_requests_1_notFull;

  // action method portalIfc_requests_2_enq
  input  [31 : 0] portalIfc_requests_2_enq_v;
  input  EN_portalIfc_requests_2_enq;
  output RDY_portalIfc_requests_2_enq;

  // value method portalIfc_requests_2_notFull
  output portalIfc_requests_2_notFull;
  output RDY_portalIfc_requests_2_notFull;

  // value method portalIfc_intr_status
  output portalIfc_intr_status;
  output RDY_portalIfc_intr_status;

  // value method portalIfc_intr_channel
  output [31 : 0] portalIfc_intr_channel;
  output RDY_portalIfc_intr_channel;

  // value method pipes_startSwitching_PipeOut_first
  output [71 : 0] pipes_startSwitching_PipeOut_first;
  output RDY_pipes_startSwitching_PipeOut_first;

  // action method pipes_startSwitching_PipeOut_deq
  input  EN_pipes_startSwitching_PipeOut_deq;
  output RDY_pipes_startSwitching_PipeOut_deq;

  // value method pipes_startSwitching_PipeOut_notEmpty
  output pipes_startSwitching_PipeOut_notEmpty;
  output RDY_pipes_startSwitching_PipeOut_notEmpty;

  // value method pipes_printSwStats_PipeOut_first
  output [31 : 0] pipes_printSwStats_PipeOut_first;
  output RDY_pipes_printSwStats_PipeOut_first;

  // action method pipes_printSwStats_PipeOut_deq
  input  EN_pipes_printSwStats_PipeOut_deq;
  output RDY_pipes_printSwStats_PipeOut_deq;

  // value method pipes_printSwStats_PipeOut_notEmpty
  output pipes_printSwStats_PipeOut_notEmpty;
  output RDY_pipes_printSwStats_PipeOut_notEmpty;

  // value method pipes_start_shoal_PipeOut_first
  output [119 : 0] pipes_start_shoal_PipeOut_first;
  output RDY_pipes_start_shoal_PipeOut_first;

  // action method pipes_start_shoal_PipeOut_deq
  input  EN_pipes_start_shoal_PipeOut_deq;
  output RDY_pipes_start_shoal_PipeOut_deq;

  // value method pipes_start_shoal_PipeOut_notEmpty
  output pipes_start_shoal_PipeOut_notEmpty;
  output RDY_pipes_start_shoal_PipeOut_notEmpty;

  // signals for module outputs
  reg [15 : 0] portalIfc_messageSize_size;
  wire [119 : 0] pipes_start_shoal_PipeOut_first;
  wire [71 : 0] pipes_startSwitching_PipeOut_first;
  wire [31 : 0] pipes_printSwStats_PipeOut_first, portalIfc_intr_channel;
  wire RDY_pipes_printSwStats_PipeOut_deq,
       RDY_pipes_printSwStats_PipeOut_first,
       RDY_pipes_printSwStats_PipeOut_notEmpty,
       RDY_pipes_startSwitching_PipeOut_deq,
       RDY_pipes_startSwitching_PipeOut_first,
       RDY_pipes_startSwitching_PipeOut_notEmpty,
       RDY_pipes_start_shoal_PipeOut_deq,
       RDY_pipes_start_shoal_PipeOut_first,
       RDY_pipes_start_shoal_PipeOut_notEmpty,
       RDY_portalIfc_intr_channel,
       RDY_portalIfc_intr_status,
       RDY_portalIfc_messageSize_size,
       RDY_portalIfc_requests_0_enq,
       RDY_portalIfc_requests_0_notFull,
       RDY_portalIfc_requests_1_enq,
       RDY_portalIfc_requests_1_notFull,
       RDY_portalIfc_requests_2_enq,
       RDY_portalIfc_requests_2_notFull,
       pipes_printSwStats_PipeOut_notEmpty,
       pipes_startSwitching_PipeOut_notEmpty,
       pipes_start_shoal_PipeOut_notEmpty,
       portalIfc_intr_status,
       portalIfc_requests_0_notFull,
       portalIfc_requests_1_notFull,
       portalIfc_requests_2_notFull;

  // register printSwStats_requestAdapter_fbnbuff
  reg [31 : 0] printSwStats_requestAdapter_fbnbuff;
  wire [31 : 0] printSwStats_requestAdapter_fbnbuff_D_IN;
  wire printSwStats_requestAdapter_fbnbuff_EN;

  // register startSwitching_requestAdapter_count
  reg [1 : 0] startSwitching_requestAdapter_count;
  wire [1 : 0] startSwitching_requestAdapter_count_D_IN;
  wire startSwitching_requestAdapter_count_EN;

  // register startSwitching_requestAdapter_fbnbuff
  reg [95 : 0] startSwitching_requestAdapter_fbnbuff;
  wire [95 : 0] startSwitching_requestAdapter_fbnbuff_D_IN;
  wire startSwitching_requestAdapter_fbnbuff_EN;

  // register start_shoal_requestAdapter_count
  reg [1 : 0] start_shoal_requestAdapter_count;
  wire [1 : 0] start_shoal_requestAdapter_count_D_IN;
  wire start_shoal_requestAdapter_count_EN;

  // register start_shoal_requestAdapter_fbnbuff
  reg [127 : 0] start_shoal_requestAdapter_fbnbuff;
  wire [127 : 0] start_shoal_requestAdapter_fbnbuff_D_IN;
  wire start_shoal_requestAdapter_fbnbuff_EN;

  // ports of submodule printSwStats_requestAdapter_fifo
  wire [31 : 0] printSwStats_requestAdapter_fifo_D_IN,
		printSwStats_requestAdapter_fifo_D_OUT;
  wire printSwStats_requestAdapter_fifo_CLR,
       printSwStats_requestAdapter_fifo_DEQ,
       printSwStats_requestAdapter_fifo_EMPTY_N,
       printSwStats_requestAdapter_fifo_ENQ,
       printSwStats_requestAdapter_fifo_FULL_N;

  // ports of submodule startSwitching_requestAdapter_fifo
  wire [71 : 0] startSwitching_requestAdapter_fifo_D_IN,
		startSwitching_requestAdapter_fifo_D_OUT;
  wire startSwitching_requestAdapter_fifo_CLR,
       startSwitching_requestAdapter_fifo_DEQ,
       startSwitching_requestAdapter_fifo_EMPTY_N,
       startSwitching_requestAdapter_fifo_ENQ,
       startSwitching_requestAdapter_fifo_FULL_N;

  // ports of submodule start_shoal_requestAdapter_fifo
  wire [119 : 0] start_shoal_requestAdapter_fifo_D_IN,
		 start_shoal_requestAdapter_fifo_D_OUT;
  wire start_shoal_requestAdapter_fifo_CLR,
       start_shoal_requestAdapter_fifo_DEQ,
       start_shoal_requestAdapter_fifo_EMPTY_N,
       start_shoal_requestAdapter_fifo_ENQ,
       start_shoal_requestAdapter_fifo_FULL_N;

  // remaining internal signals
  wire [1 : 0] x__h1051, x__h825;

  // value method portalIfc_messageSize_size
  always@(portalIfc_messageSize_size_methodNumber)
  begin
    case (portalIfc_messageSize_size_methodNumber)
      16'd0: portalIfc_messageSize_size = 16'd72;
      16'd1: portalIfc_messageSize_size = 16'd32;
      default: portalIfc_messageSize_size = 16'd120;
    endcase
  end
  assign RDY_portalIfc_messageSize_size = 1'd1 ;

  // action method portalIfc_requests_0_enq
  assign RDY_portalIfc_requests_0_enq =
	     startSwitching_requestAdapter_count < 2'd2 ||
	     startSwitching_requestAdapter_fifo_FULL_N ;

  // value method portalIfc_requests_0_notFull
  assign portalIfc_requests_0_notFull =
	     startSwitching_requestAdapter_fifo_FULL_N ;
  assign RDY_portalIfc_requests_0_notFull = 1'd1 ;

  // action method portalIfc_requests_1_enq
  assign RDY_portalIfc_requests_1_enq =
	     printSwStats_requestAdapter_fifo_FULL_N ;

  // value method portalIfc_requests_1_notFull
  assign portalIfc_requests_1_notFull =
	     printSwStats_requestAdapter_fifo_FULL_N ;
  assign RDY_portalIfc_requests_1_notFull = 1'd1 ;

  // action method portalIfc_requests_2_enq
  assign RDY_portalIfc_requests_2_enq =
	     start_shoal_requestAdapter_count != 2'd3 ||
	     start_shoal_requestAdapter_fifo_FULL_N ;

  // value method portalIfc_requests_2_notFull
  assign portalIfc_requests_2_notFull =
	     start_shoal_requestAdapter_fifo_FULL_N ;
  assign RDY_portalIfc_requests_2_notFull = 1'd1 ;

  // value method portalIfc_intr_status
  assign portalIfc_intr_status = 1'd0 ;
  assign RDY_portalIfc_intr_status = 1'd1 ;

  // value method portalIfc_intr_channel
  assign portalIfc_intr_channel = 32'hFFFFFFFF ;
  assign RDY_portalIfc_intr_channel = 1'd1 ;

  // value method pipes_startSwitching_PipeOut_first
  assign pipes_startSwitching_PipeOut_first =
	     startSwitching_requestAdapter_fifo_D_OUT ;
  assign RDY_pipes_startSwitching_PipeOut_first =
	     startSwitching_requestAdapter_fifo_EMPTY_N ;

  // action method pipes_startSwitching_PipeOut_deq
  assign RDY_pipes_startSwitching_PipeOut_deq =
	     startSwitching_requestAdapter_fifo_EMPTY_N ;

  // value method pipes_startSwitching_PipeOut_notEmpty
  assign pipes_startSwitching_PipeOut_notEmpty =
	     startSwitching_requestAdapter_fifo_EMPTY_N ;
  assign RDY_pipes_startSwitching_PipeOut_notEmpty = 1'd1 ;

  // value method pipes_printSwStats_PipeOut_first
  assign pipes_printSwStats_PipeOut_first =
	     printSwStats_requestAdapter_fifo_D_OUT ;
  assign RDY_pipes_printSwStats_PipeOut_first =
	     printSwStats_requestAdapter_fifo_EMPTY_N ;

  // action method pipes_printSwStats_PipeOut_deq
  assign RDY_pipes_printSwStats_PipeOut_deq =
	     printSwStats_requestAdapter_fifo_EMPTY_N ;

  // value method pipes_printSwStats_PipeOut_notEmpty
  assign pipes_printSwStats_PipeOut_notEmpty =
	     printSwStats_requestAdapter_fifo_EMPTY_N ;
  assign RDY_pipes_printSwStats_PipeOut_notEmpty = 1'd1 ;

  // value method pipes_start_shoal_PipeOut_first
  assign pipes_start_shoal_PipeOut_first =
	     start_shoal_requestAdapter_fifo_D_OUT ;
  assign RDY_pipes_start_shoal_PipeOut_first =
	     start_shoal_requestAdapter_fifo_EMPTY_N ;

  // action method pipes_start_shoal_PipeOut_deq
  assign RDY_pipes_start_shoal_PipeOut_deq =
	     start_shoal_requestAdapter_fifo_EMPTY_N ;

  // value method pipes_start_shoal_PipeOut_notEmpty
  assign pipes_start_shoal_PipeOut_notEmpty =
	     start_shoal_requestAdapter_fifo_EMPTY_N ;
  assign RDY_pipes_start_shoal_PipeOut_notEmpty = 1'd1 ;

  // submodule printSwStats_requestAdapter_fifo
  FIFO1 #(.width(32'd32),
	  .guarded(1'd1)) printSwStats_requestAdapter_fifo(.RST(RST_N),
							   .CLK(CLK),
							   .D_IN(printSwStats_requestAdapter_fifo_D_IN),
							   .ENQ(printSwStats_requestAdapter_fifo_ENQ),
							   .DEQ(printSwStats_requestAdapter_fifo_DEQ),
							   .CLR(printSwStats_requestAdapter_fifo_CLR),
							   .D_OUT(printSwStats_requestAdapter_fifo_D_OUT),
							   .FULL_N(printSwStats_requestAdapter_fifo_FULL_N),
							   .EMPTY_N(printSwStats_requestAdapter_fifo_EMPTY_N));

  // submodule startSwitching_requestAdapter_fifo
  FIFO1 #(.width(32'd72),
	  .guarded(1'd1)) startSwitching_requestAdapter_fifo(.RST(RST_N),
							     .CLK(CLK),
							     .D_IN(startSwitching_requestAdapter_fifo_D_IN),
							     .ENQ(startSwitching_requestAdapter_fifo_ENQ),
							     .DEQ(startSwitching_requestAdapter_fifo_DEQ),
							     .CLR(startSwitching_requestAdapter_fifo_CLR),
							     .D_OUT(startSwitching_requestAdapter_fifo_D_OUT),
							     .FULL_N(startSwitching_requestAdapter_fifo_FULL_N),
							     .EMPTY_N(startSwitching_requestAdapter_fifo_EMPTY_N));

  // submodule start_shoal_requestAdapter_fifo
  FIFO1 #(.width(32'd120),
	  .guarded(1'd1)) start_shoal_requestAdapter_fifo(.RST(RST_N),
							  .CLK(CLK),
							  .D_IN(start_shoal_requestAdapter_fifo_D_IN),
							  .ENQ(start_shoal_requestAdapter_fifo_ENQ),
							  .DEQ(start_shoal_requestAdapter_fifo_DEQ),
							  .CLR(start_shoal_requestAdapter_fifo_CLR),
							  .D_OUT(start_shoal_requestAdapter_fifo_D_OUT),
							  .FULL_N(start_shoal_requestAdapter_fifo_FULL_N),
							  .EMPTY_N(start_shoal_requestAdapter_fifo_EMPTY_N));

  // register printSwStats_requestAdapter_fbnbuff
  assign printSwStats_requestAdapter_fbnbuff_D_IN =
	     portalIfc_requests_1_enq_v ;
  assign printSwStats_requestAdapter_fbnbuff_EN =
	     EN_portalIfc_requests_1_enq ;

  // register startSwitching_requestAdapter_count
  assign startSwitching_requestAdapter_count_D_IN =
	     (startSwitching_requestAdapter_count == 2'd2) ? 2'd0 : x__h825 ;
  assign startSwitching_requestAdapter_count_EN =
	     EN_portalIfc_requests_0_enq ;

  // register startSwitching_requestAdapter_fbnbuff
  assign startSwitching_requestAdapter_fbnbuff_D_IN =
	     { startSwitching_requestAdapter_fbnbuff[63:0],
	       portalIfc_requests_0_enq_v } ;
  assign startSwitching_requestAdapter_fbnbuff_EN =
	     EN_portalIfc_requests_0_enq ;

  // register start_shoal_requestAdapter_count
  assign start_shoal_requestAdapter_count_D_IN =
	     (start_shoal_requestAdapter_count == 2'd3) ? 2'd0 : x__h1051 ;
  assign start_shoal_requestAdapter_count_EN = EN_portalIfc_requests_2_enq ;

  // register start_shoal_requestAdapter_fbnbuff
  assign start_shoal_requestAdapter_fbnbuff_D_IN =
	     { start_shoal_requestAdapter_fbnbuff[95:0],
	       portalIfc_requests_2_enq_v } ;
  assign start_shoal_requestAdapter_fbnbuff_EN = EN_portalIfc_requests_2_enq ;

  // submodule printSwStats_requestAdapter_fifo
  assign printSwStats_requestAdapter_fifo_D_IN = portalIfc_requests_1_enq_v ;
  assign printSwStats_requestAdapter_fifo_ENQ = EN_portalIfc_requests_1_enq ;
  assign printSwStats_requestAdapter_fifo_DEQ =
	     EN_pipes_printSwStats_PipeOut_deq ;
  assign printSwStats_requestAdapter_fifo_CLR = 1'b0 ;

  // submodule startSwitching_requestAdapter_fifo
  assign startSwitching_requestAdapter_fifo_D_IN =
	     { startSwitching_requestAdapter_fbnbuff[39:0],
	       portalIfc_requests_0_enq_v } ;
  assign startSwitching_requestAdapter_fifo_ENQ =
	     EN_portalIfc_requests_0_enq &&
	     startSwitching_requestAdapter_count == 2'd2 ;
  assign startSwitching_requestAdapter_fifo_DEQ =
	     EN_pipes_startSwitching_PipeOut_deq ;
  assign startSwitching_requestAdapter_fifo_CLR = 1'b0 ;

  // submodule start_shoal_requestAdapter_fifo
  assign start_shoal_requestAdapter_fifo_D_IN =
	     { start_shoal_requestAdapter_fbnbuff[87:0],
	       portalIfc_requests_2_enq_v } ;
  assign start_shoal_requestAdapter_fifo_ENQ =
	     EN_portalIfc_requests_2_enq &&
	     start_shoal_requestAdapter_count == 2'd3 ;
  assign start_shoal_requestAdapter_fifo_DEQ =
	     EN_pipes_start_shoal_PipeOut_deq ;
  assign start_shoal_requestAdapter_fifo_CLR = 1'b0 ;

  // remaining internal signals
  assign x__h1051 = start_shoal_requestAdapter_count + 2'd1 ;
  assign x__h825 = startSwitching_requestAdapter_count + 2'd1 ;

  // handling of inlined registers

  always@(posedge CLK)
  begin
    if (RST_N == `BSV_RESET_VALUE)
      begin
        printSwStats_requestAdapter_fbnbuff <= `BSV_ASSIGNMENT_DELAY 32'd0;
	startSwitching_requestAdapter_count <= `BSV_ASSIGNMENT_DELAY 2'd0;
	startSwitching_requestAdapter_fbnbuff <= `BSV_ASSIGNMENT_DELAY 96'd0;
	start_shoal_requestAdapter_count <= `BSV_ASSIGNMENT_DELAY 2'd0;
	start_shoal_requestAdapter_fbnbuff <= `BSV_ASSIGNMENT_DELAY 128'd0;
      end
    else
      begin
        if (printSwStats_requestAdapter_fbnbuff_EN)
	  printSwStats_requestAdapter_fbnbuff <= `BSV_ASSIGNMENT_DELAY
	      printSwStats_requestAdapter_fbnbuff_D_IN;
	if (startSwitching_requestAdapter_count_EN)
	  startSwitching_requestAdapter_count <= `BSV_ASSIGNMENT_DELAY
	      startSwitching_requestAdapter_count_D_IN;
	if (startSwitching_requestAdapter_fbnbuff_EN)
	  startSwitching_requestAdapter_fbnbuff <= `BSV_ASSIGNMENT_DELAY
	      startSwitching_requestAdapter_fbnbuff_D_IN;
	if (start_shoal_requestAdapter_count_EN)
	  start_shoal_requestAdapter_count <= `BSV_ASSIGNMENT_DELAY
	      start_shoal_requestAdapter_count_D_IN;
	if (start_shoal_requestAdapter_fbnbuff_EN)
	  start_shoal_requestAdapter_fbnbuff <= `BSV_ASSIGNMENT_DELAY
	      start_shoal_requestAdapter_fbnbuff_D_IN;
      end
  end

  // synopsys translate_off
  `ifdef BSV_NO_INITIAL_BLOCKS
  `else // not BSV_NO_INITIAL_BLOCKS
  initial
  begin
    printSwStats_requestAdapter_fbnbuff = 32'hAAAAAAAA;
    startSwitching_requestAdapter_count = 2'h2;
    startSwitching_requestAdapter_fbnbuff = 96'hAAAAAAAAAAAAAAAAAAAAAAAA;
    start_shoal_requestAdapter_count = 2'h2;
    start_shoal_requestAdapter_fbnbuff =
	128'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
  end
  `endif // BSV_NO_INITIAL_BLOCKS
  // synopsys translate_on
endmodule  // mkShoalMultiSimTopRequestInput

