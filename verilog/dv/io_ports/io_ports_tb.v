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

`timescale 1 ns / 1 ps

`define MEMFILE "hello.hex"
`define MEM_SIZE 64*4            // Memory size in Byte

module io_ports_tb;
   reg clock;
   reg RSTB;
   reg CSB;
   reg power1, power2;
   reg power3, power4;
   
   wire gpio;
   wire [37:0] mprj_io;
   wire [7:0]  mprj_io_counter;
   wire        mprj_io_37;
   wire        mprj_io_25;
   wire        mprj_io_24;
   wire [3:0]  mprj_io_led;
   
   reg 	       ext_rst_en;
   reg 	       ext_rst;
   reg 	       ext_clk_en;
   
   reg 	       proc_sim_finish = 1'b0;
   reg 	       sim_finish = 1'b0;
   
   reg 	       done_flag = 1'b0;
   
   assign mprj_io[26] = ext_rst_en;
   assign mprj_io[27] = ext_rst;
   assign mprj_io[28] = ext_clk_en;
   assign mprj_io[29] = clock;
   
   assign mprj_io_counter = mprj_io[23:16];
   assign mprj_io_37 = mprj_io[37];
   assign mprj_io_25 = mprj_io[25];
   assign mprj_io_24 = mprj_io[24];
   assign mprj_io_led = mprj_io[33:30];
   
   assign mprj_io[3] = (CSB == 1'b1) ? 1'b1 : 1'bz;
   // assign mprj_io[3] = 1'b1;
   
   // External clock is used by default.  Make this artificially fast for the
   // simulation.  Normally this would be a slow clock and the digital PLL
   // would be the fast clock.
   
   always #12.5 clock <= (clock === 1'b0);
   
   initial begin
      clock = 0;
   end
   
`ifdef ENABLE_SDF
   initial begin
      $sdf_annotate("../../../sdf/user_proj_example.sdf", uut.mprj) ;
      $sdf_annotate("../../../sdf/user_project_wrapper.sdf", uut.mprj.mprj) ;
      $sdf_annotate("../../../mgmt_core_wrapper/sdf/DFFRAM.sdf", uut.soc.DFFRAM_0) ;
      $sdf_annotate("../../../mgmt_core_wrapper/sdf/mgmt_core.sdf", uut.soc.core) ;
      $sdf_annotate("../../../caravel/sdf/housekeeping.sdf", uut.housekeeping) ;
      $sdf_annotate("../../../caravel/sdf/chip_io.sdf", uut.padframe) ;
      $sdf_annotate("../../../caravel/sdf/mprj_logic_high.sdf", uut.mgmt_buffers.mprj_logic_high_inst) ;
      $sdf_annotate("../../../caravel/sdf/mprj2_logic_high.sdf", uut.mgmt_buffers.mprj2_logic_high_inst) ;
      $sdf_annotate("../../../caravel/sdf/mgmt_protect_hv.sdf", uut.mgmt_buffers.powergood_check) ;
      $sdf_annotate("../../../caravel/sdf/mgmt_protect.sdf", uut.mgmt_buffers) ;
      $sdf_annotate("../../../caravel/sdf/caravel_clocking.sdf", uut.clocking) ;
      $sdf_annotate("../../../caravel/sdf/digital_pll.sdf", uut.pll) ;
      $sdf_annotate("../../../caravel/sdf/xres_buf.sdf", uut.rstb_level) ;
      $sdf_annotate("../../../caravel/sdf/user_id_programming.sdf", uut.user_id_value) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_bidir_1[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_bidir_1[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_bidir_2[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_bidir_2[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_bidir_2[2] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[2] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[3] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[4] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[5] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[6] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[7] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[8] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[9] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1[10] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1a[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1a[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1a[2] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1a[3] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1a[4] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_1a[5] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[2] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[3] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[4] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[5] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[6] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[7] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[8] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[9] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[10] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[11] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[12] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[13] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[14] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_control_block.sdf", uut.\gpio_control_in_2[15] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.\gpio_defaults_block_0[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.\gpio_defaults_block_0[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.\gpio_defaults_block_2[0] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.\gpio_defaults_block_2[1] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.\gpio_defaults_block_2[2] ) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_5) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_6) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_7) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_8) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_9) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_10) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_11) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_12) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_13) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_14) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_15) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_16) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_17) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_18) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_19) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_20) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_21) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_22) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_23) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_24) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_25) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_26) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_27) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_28) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_29) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_30) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_31) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_32) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_33) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_34) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_35) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_36) ;
      $sdf_annotate("../../../caravel/sdf/gpio_defaults_block.sdf", uut.gpio_defaults_block_37) ;
   end
`endif 

   initial begin
      $dumpfile("io_ports.vcd");
      $dumpvars(0, io_ports_tb);
      
      // Repeat cycles of 1000 clock edges as needed to complete testbench
      repeat (100) begin
	 repeat (1000) @(posedge clock);
	 // $display("+1000 cycles");
      end
      $display("%c[1;31m",27);
`ifdef GL
      $display ("Monitor: Timeout, Test Mega-Project IO Ports (GL) Failed");
`else
      $display ("Monitor: Timeout, Test Mega-Project IO Ports (RTL) Failed");
`endif
      $display("%c[0m",27);
      $finish;
   end
   
   initial begin
      ext_clk_en <= 1'b1; // external clock input enable
      ext_rst_en <= 1'b1; // external reset input enable
      ext_rst <= 1'b1; // external reset input
      
      wait(done_flag == 1'b1);
      
      #100;
      
      ext_rst <= 1'b0; // external reset input
      wait(sim_finish);
      
`ifdef GL
      $display("Monitor: Test 1 Mega-Project IO (GL) Passed");
`else
      $display("Monitor: Test 1 Mega-Project IO (RTL) Passed");
`endif
      
      $write("\nSimulation Finish!\n"                     );
      $write("Simulation Clock Cycles    : %10d\n", cntr   );
      $write("Processor  Clock Cycles    : %10d\n", cycle  );
`ifndef GL
      $write("Valid Instructions Executed: %10d\n", instret);
`endif
      $write("\n");
      
      $finish;
   end
   
   initial begin
      RSTB <= 1'b0;
      CSB  <= 1'b1;		// Force CSB high
      #2000;
      RSTB <= 1'b1;	    	// Release reset
      #3_00_000;
      CSB = 1'b0;		// CSB can be released
   end
   
   initial begin		// Power-up sequence
      power1 <= 1'b0;
      power2 <= 1'b0;
      power3 <= 1'b0;
      power4 <= 1'b0;
      #100;
      power1 <= 1'b1;
      #100;
      power2 <= 1'b1;
      #100;
      power3 <= 1'b1;
      #100;
      power4 <= 1'b1;
   end
   
   wire flash_csb;
   wire flash_clk;
   wire flash_io0;
   wire flash_io1;
   
   wire VDD3V3;
   wire VDD1V8;
   wire VSS;
   
   assign VDD3V3 = power1;
   assign VDD1V8 = power2;
   assign VSS = 1'b0;
   
   caravel uut (.VDD	  (VDD3V3),
		.VSS	  (VSS),
		.clock    (clock),
		.gpio     (gpio),
		.mprj_io  (mprj_io),
		.flash_csb(flash_csb),
		.flash_clk(flash_clk),
		.flash_io0(flash_io0),
		.flash_io1(flash_io1),
		.resetb	  (RSTB));
   
   spiflash #(.FILENAME("io_ports.hex")) spiflash (.csb(flash_csb),
						   .clk(flash_clk),
						   .io0(flash_io0),
						   .io1(flash_io1),
						   .io2(),			// not used
						   .io3()			// not used
						   );

   always @(posedge clock) begin
      if ((done_flag == 1'b0) && (mprj_io_37 == 1'b0)) begin
	 done_flag <= 1'b1;
	 $display(">>>>> IO setup done");
      end
   end

    // counter
    reg [31:0] cntr;
    always @(posedge clock) begin
        if (ext_rst) begin
            cntr <= 0;
        end else begin
            cntr <= cntr+1;
        end
    end

    // send program
    reg [31:0] rom [0:(`MEM_SIZE/4)-1];
    initial begin
       $readmemh(`MEMFILE, rom);
    end

    reg      [$clog2(`MEM_SIZE):0] addr    , n_addr;
    wire [$clog2(`MEM_SIZE/4)-1:0] index           ;
    wire                     [1:0] offset          ;
    wire                     [7:0] tx_data         ;
    wire                           tx_we           ;
    wire                           tx_ready        ;
    reg                            r_tx_we         ;

    assign index   = addr[$clog2(`MEM_SIZE)-1:2]               ;
    assign offset  = addr[1:0]                                 ;
    assign tx_data = rom[index][offset*8+:8]                   ;
    assign tx_we   = (!r_tx_we && tx_ready && (addr<`MEM_SIZE));

    always @(*) begin
        n_addr = addr;
        if (tx_we) begin
            n_addr = addr+1;
        end
    end

    always @(posedge clock) begin
        if (ext_rst) begin
            addr    <= 0   ;
            r_tx_we <= 1'b0;
        end else begin
            addr    <= n_addr;
            r_tx_we <= tx_we ;
        end
    end

    // uart transmitter
    wire txd;
    wire rxd;
    UartTx uart_tx0 (
        .CLK  (clock     ),
        .RST_X(~ext_rst   ),
        .DATA (tx_data ),
        .WE   (tx_we   ),
        .TXD  (txd     ),
        .READY(tx_ready)
    );

    // processor
   assign mprj_io[35] = txd;
   assign rxd = mprj_io[34];
   wire [3:0] led;
   assign led = mprj_io[33:30];

    // uart receiver
    wire [7:0] rx_data;
    wire       rx_re  ;
    serialc serialc0 (
        .CLK  (clock    ),
        .RST_X(~ext_rst  ),
        .RXD  (rxd    ),
        .DATA (rx_data),
        .EN   (rx_re  )
    );

    // uart putchar
    always @(negedge clock) begin
        if (rx_re) begin
	   $write("%c", rx_data);
	end
    end

`ifdef GL
    // counter
    reg [31:0] cycle  ;
    always @(posedge clock) begin
        if (ext_rst) begin
            cycle   <= 0;
        end 
	else begin
	   cycle <= cycle+1;
	   if (cycle >= 55000)
             sim_finish <= 1'b1;
	end
    end
`else
    // simulation finish
    always @(negedge clock) begin
        if (uut.chip_core.mprj.mprj.i_rvcorep.tohost_we && (uut.chip_core.mprj.mprj.i_rvcorep.tohost_data[17:16]==2'b10)) begin
            proc_sim_finish <= 1'b1;
        end
        if (proc_sim_finish && (uut.chip_core.mprj.mprj.i_rvcorep.queue_num==0) && rx_re) begin
            sim_finish      <= 1'b1;
        end
    end

    // counter
    reg [31:0] cycle  ;
    reg [31:0] instret;
    always @(posedge clock) begin
        if (uut.chip_core.mprj.mprj.i_rvcorep.p.r_rst) begin
            cycle   <= 0;
            instret <= 0;
        end else if (!proc_sim_finish) begin
            cycle   <= cycle+1;
            instret <= instret+uut.chip_core.mprj.mprj.i_rvcorep.p.MaWb_v;
        end
    end
`endif

    // simulation output

   always @(cntr) begin
      if (cntr%10000 == 0 && ext_rst == 0) begin
	 $display("-- RVcore Clock Cycles: %d", cntr);
	 $fflush();
      end
   end
   
   always @(negedge ext_rst) begin
      $display("%s loaded from file", `MEMFILE);
      $display("RVcore started");
      $fflush();
   end

endmodule

`ifdef GL
/********************************************************************************************/
`define SERIAL_WCNT  20          // 100MHz clock and 5Mbaud -> 20
/********************************************************************************************/
module UartTx(CLK, RST_X, DATA, WE, TXD, READY);
    input wire       CLK, RST_X, WE;
    input wire [7:0] DATA;
    output reg       TXD, READY;

    reg [8:0]   cmd;
    reg [11:0]  waitnum;
    reg [3:0]   cnt;

    always @(posedge CLK) begin
        if(!RST_X) begin
            TXD       <= 1'b1;
            READY     <= 1'b1;
            cmd       <= 9'h1ff;
            waitnum   <= 0;
            cnt       <= 0;
        end else if( READY ) begin
            TXD       <= 1'b1;
            waitnum   <= 0;
            if( WE )begin
                READY <= 1'b0;
                cmd   <= {DATA, 1'b0};
                cnt   <= 10;
            end
        end else if( waitnum >= `SERIAL_WCNT ) begin
            TXD       <= cmd[0];
            READY     <= (cnt == 1);
            cmd       <= {1'b1, cmd[8:1]};
            waitnum   <= 1;
            cnt       <= cnt - 1;
        end else begin
            waitnum   <= waitnum + 1;
        end
    end
endmodule

/********************************************************************************************/
/* RS232C serial controller (deserializer):                                                 */
/********************************************************************************************/
`define SS_SER_WAIT  'd0         // RS232C deserializer, State WAIT
`define SS_SER_RCV0  'd1         // RS232C deserializer, State Receive 0th bit
                                 // States Receive 1st bit to 7th bit are not used
`define SS_SER_DONE  'd9         // RS232C deserializer, State DONE
/********************************************************************************************/
module serialc(CLK, RST_X, RXD, DATA, EN);
    input  wire    CLK, RST_X, RXD; // clock, reset, RS232C input
    output [7:0]   DATA;            // 8bit output data
    output reg     EN;              // 8bit output data enable

    reg    [7:0]   DATA;
    reg    [3:0]   stage;
    reg    [12:0]  cnt;             // counter to latch D0, D1, ..., D7
    reg    [11:0]  cnt_start;       // counter to detect the Start Bit
    
    wire   [12:0]  waitcnt;
    assign waitcnt = `SERIAL_WCNT;

    always @(posedge CLK)
      if (!RST_X) cnt_start <= 0;
      else        cnt_start <= (RXD) ? 0 : cnt_start + 1;

    always @(posedge CLK)
      if(!RST_X) begin
          EN     <= 0;
          stage  <= `SS_SER_WAIT;
          cnt    <= 1;
          DATA   <= 0;
      end else if (stage == `SS_SER_WAIT) begin // detect the Start Bit
          EN <= 0;
          stage <= (cnt_start == (waitcnt >> 1)) ? `SS_SER_RCV0 : stage;
      end else begin
          if (cnt != waitcnt) begin
              cnt <= cnt + 1;
              EN <= 0;
          end else begin               // receive 1bit data
              stage  <= (stage == `SS_SER_DONE) ? `SS_SER_WAIT : stage + 1;
              EN     <= (stage == 8)  ? 1 : 0;
              DATA   <= {RXD, DATA[7:1]};
              cnt <= 1;
          end
      end
endmodule
`endif

`default_nettype wire
