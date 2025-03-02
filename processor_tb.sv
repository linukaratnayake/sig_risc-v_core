`timescale 10ns / 1ps

module processor_tb
	#(
	parameter BUS_WIDTH = 32
	);

    reg clk = 1'b0;

    // Clock generation
    always begin
        #10 clk = ~clk;
    end
	 
	localparam DATA_MEMORY_ADDR_BUS_WIDTH = 32;
    localparam DATA_MEMORY_DATA_BUS_WIDTH = 32;
    localparam REG_FILE_ADDR_BUS_WIDTH = 5;
    localparam REG_FILE_DATA_BUS_WIDTH = 32;
    localparam INST_MEMORY_ADDR_BUS_WIDTH = 32;
    localparam INST_MEMORY_DATA_BUS_WIDTH = 32;
     
    // Wires for module instantiation, and connections
    wire [INST_MEMORY_ADDR_BUS_WIDTH - 1:0] pc_out;
    wire [INST_MEMORY_DATA_BUS_WIDTH - 1:0] instr;
    wire [DATA_MEMORY_DATA_BUS_WIDTH - 1:0] read_data;
    wire [BUS_WIDTH - 1:0] pc_next;

    wire [31:0] memory_address;
    wire [31:0] data_to_write;
    wire write_data_enable;

    // Instantiate core
    core core_inst (
        .clk(clk),
        .rst(1'b0),
        .instruction(instr),
        .pc(pc_out),
        .memory_address(memory_address),
        .data_to_write(data_to_write),
        .func3(),
        .write_data(write_data_enable),
        .read_data(read_data),
        .next_pc(pc_next)
    );

    // Instantiate program counter
    pc # (INST_MEMORY_ADDR_BUS_WIDTH) pc_inst (
        .clk(clk),
        .pc_next(pc_next),
        .pc(pc_out)
    );

    // Instantiate instruction memory module
    imem # (INST_MEMORY_ADDR_BUS_WIDTH, INST_MEMORY_DATA_BUS_WIDTH) imem_inst (
        .a(pc_out),
        .rd(instr)
    );

    // Instantiate data_memory module
    data_memory #(DATA_MEMORY_ADDR_BUS_WIDTH, DATA_MEMORY_DATA_BUS_WIDTH) data_memory_inst (
        .clk(clk),
        .addr(memory_address - 32'h00002000),
        .write_data(data_to_write),
        .write_en(write_data_enable),
        .read_data(read_data)
    );

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, processor_tb);
        #1000 $finish();
    end
endmodule

module imem
    #(  
        parameter ADDR_BUS_WIDTH = 16,
        parameter DATA_BUS_WIDTH = 32
    )
    (
        input wire [ADDR_BUS_WIDTH - 1:0] a,
        output wire [DATA_BUS_WIDTH - 1:0] rd
    );

    localparam MEM_DEPTH = 2 ** (ADDR_BUS_WIDTH - 10);
    localparam MEM_WIDTH = 8;
      
    reg [MEM_WIDTH - 1:0] memory[MEM_DEPTH - 1:0];

	initial
	begin
		// lw x6 -4(x9)
		memory[4] = 8'hFF;
		memory[5] = 8'hC4;
		memory[6] = 8'hA3;
		memory[7] = 8'h03;
		
		// sw x6, 8(x9)
		memory[8] = 8'h00;
		memory[9] = 8'h64;
		memory[10] = 8'hA4;
		memory[11] = 8'h23;

		// or x4, x5, x6
        memory[12] = 8'h00;
		memory[13] = 8'h62;
		memory[14] = 8'hE2;
		memory[15] = 8'h33;

		// beq x4, x4, 24
        memory[16] = 8'h00;  // 8'hFE;
		memory[17] = 8'h42;
		memory[18] = 8'h0C;  // 8'h0A;
		memory[19] = 8'h63;  // 8'hE3;

		// or x4, x5, x6
		memory[40] = 8'h00;
		memory[41] = 8'h62;
		memory[42] = 8'hE2;
		memory[43] = 8'h33;

		// Addi x12, x5(17)
		memory[44] = 8'h01;
		memory[45] = 8'h12;
		memory[46] = 8'h86;
		memory[47] = 8'h13;

		// lui x7, 4560
		memory[48] = 8'h01;
		memory[49] = 8'h1d;
		memory[50] = 8'h03;
		memory[51] = 8'hb7;

		// jal x2, -40
		memory[52] = 8'hfd;
		memory[53] = 8'h9f;
		memory[54] = 8'hf1;
		memory[55] = 8'h6f;
	end
        
   assign rd = {memory[a], memory[a + 1], memory[a + 2], memory[a + 3]};
endmodule

module pc
    #(parameter BUS_WIDTH = 32)
    (
        input clk,
        input [BUS_WIDTH - 1:0] pc_next,
        output reg [BUS_WIDTH - 1:0] pc
    );
	 
	 initial
	 begin
		  pc = 16'h0000;
	 end
    
    always @(posedge clk)
    begin
        pc <= pc_next;
    end
endmodule

module data_memory
    #(  
        parameter ADDR_BUS_WIDTH = 32,
        parameter DATA_BUS_WIDTH = 32)
    (
        input clk,
        input [ADDR_BUS_WIDTH - 1:0] addr,
        input [DATA_BUS_WIDTH - 1:0] write_data,
        input write_en,
        output [DATA_BUS_WIDTH - 1:0] read_data
    );
        
    localparam MEM_DEPTH = 64;
    localparam MEM_WIDTH = 8;

    reg [MEM_WIDTH - 1:0] mem [0:MEM_DEPTH - 1];

    assign read_data = {mem[addr], mem[addr + 1], mem[addr + 2], mem[addr + 3]};
	 
	 initial
	 begin
		mem[0] = 8'h00;
		mem[1] = 8'h00;
		mem[2] = 8'h00;
		mem[3] = 8'd10;
	 end

    always @(negedge clk)
    begin
        if(write_en)
        begin
            mem[addr] <= write_data[4 * MEM_WIDTH - 1: 3 * MEM_WIDTH];
            mem[addr + 1] <= write_data[3 * MEM_WIDTH - 1: 2 * MEM_WIDTH];
            mem[addr + 2] <= write_data[2 * MEM_WIDTH - 1: MEM_WIDTH];
            mem[addr + 3] <= write_data[MEM_WIDTH - 1: 0];
        end
    end    
endmodule
