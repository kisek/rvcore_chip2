// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 16
)(
`ifdef USE_POWER_PINS
    inout vdd,	// User area 1 1.8V supply
    inout vss,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [63:0] la_data_in,
    output [63:0] la_data_out,
    input  [63:0] la_oenb,

    // IOs
    input  [19:0] io_in,
    output [19:0] io_out,
    output [19:0] io_oeb,

    // IRQ
    output [2:0] irq
);

   // Unused
   assign wbs_ack_o = 1'b0;
   assign wbs_dat_o = 32'b0;
   assign la_data_out = 64'b0;
   
    wire clk;
    wire rst;

    wire [BITS-1:0] count;

    // IO
    assign io_out[7:0] = count[15:8];
    assign io_oeb[7:0] = 8'b0;

    // IRQ
    assign irq = 3'b000;	// Unused

   //=======================================

   wire 	    clk2;
   wire 	    rst2;
   wire 	    rxd;
   wire 	    txd;
   wire [3:0] 	    led;
   wire 	    ext_clk;
   wire 	    ext_clk_en;
   wire 	    ext_rst;
   wire 	    ext_rst_en;

   assign io_oeb[19] = 1;
   assign io_oeb[18:14] = 0;
   assign io_oeb[13:10] = 1;
   assign io_oeb[9:8] = 0;

   assign rxd = io_in[19];       // input rxd
   assign io_out[19] = 1'b0;     // (not used)

   assign io_out[18] = txd;      // ouptput txd

   assign io_out[17:14] = led;   // output led[3:0]

   assign ext_clk = io_in[13];   // external clock input
   assign io_out[13] = 1'b0;     // (not used)

   assign ext_clk_en = io_in[12];// external clock input enable
   assign io_out[12] = 1'b0;     // (not used)
                             
   assign ext_rst = io_in[11];   // external reset input
   assign io_out[11] = 1'b0;     // (not used)

   assign ext_rst_en = io_in[10];// external reset input enable
   assign io_out[10] = 1'b0;     // (not used)

   assign io_out[9] = wb_clk_i; // moniter of wb_clk_i
   
   assign io_out[8] = wb_rst_i; // moniter of wb_rst_i

   assign clk2 = (ext_clk_en) ? ext_clk: wb_clk_i;
   assign rst2 = (ext_rst_en) ? ext_rst: wb_rst_i;

   rvcorep i_rvcorep(.clk(clk2), 
                     .rst(rst2), 
                     .w_rxd(rxd),
                     .r_txd(txd), 
                     .led(led));

   counter #(.BITS(BITS)) counter(.clk(clk2),
				  .reset(rst2),
				  .count(count)
				  );
   
endmodule

module counter #(
    parameter BITS = 16
)(
    input clk,
    input reset,
    output reg [BITS-1:0] count
);

    always @(posedge clk) begin
       if (reset) count <= 0;
       else count <= count + 1;
    end

endmodule
`default_nettype wire
