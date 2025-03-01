module core (
	input clk,	// clock input
	input rst,	// reset (active high)
	input [31:0] instruction, // you need to execute this instruction
	input [31:0] pc, // the pc of the instruction that needs to execute
	
	output [31:0] memory_address, // memory address to read or write
	output [31:0] data_to_write, // data to write for store instructions
	output [2:0] func3, // simply the func3 of the store instruction 
	output write_data, // assert high to write to memory
	input [31:0] read_data, // byte-aligned read back of the address memory_address
	
	output [31:0] next_pc // pc of next instruction to execute
);
	// wires and outputs are initialized with default value so
	// that the simulations work.

	// register file
	// reg [31:0] register_file [31:0]

	// just provide rs1 + immediate
	// When the address is given the data corresponding to that address
	// will be on read_data. This data will be byte-aligned. Please search
	// this term before implmenting and load instructions
	assign memory_address = alu_result;
	// just provide rs2
	assign data_to_write = read_data_2;
	// just provide the func3 of the instruction
	assign func3 = instruction[14:12];

	assign write_data = mem_write;

	localparam BUS_WIDTH = 32;

	localparam DATA_MEMORY_ADDR_BUS_WIDTH = 32;
    localparam DATA_MEMORY_DATA_BUS_WIDTH = 32;
    localparam REG_FILE_ADDR_BUS_WIDTH = 5;
    localparam REG_FILE_DATA_BUS_WIDTH = 32;
    localparam INST_MEMORY_ADDR_BUS_WIDTH = 32;
    localparam INST_MEMORY_DATA_BUS_WIDTH = 32;
     
    // Wires for module instantiation, and connections
    wire [INST_MEMORY_ADDR_BUS_WIDTH - 1:0] pc_4;
    wire [BUS_WIDTH - 1:0] imm_ext;
    wire [DATA_MEMORY_ADDR_BUS_WIDTH - 1:0] alu_result;
    wire [REG_FILE_DATA_BUS_WIDTH - 1:0] read_data_1;
    wire [REG_FILE_DATA_BUS_WIDTH - 1:0] read_data_2;
	wire [REG_FILE_DATA_BUS_WIDTH - 1:0] write_data_out;
    wire [BUS_WIDTH - 1:0] src_a;
    wire [BUS_WIDTH - 1:0] src_b;
    wire [BUS_WIDTH - 1:0] pc_target;

    wire zero;
    wire pc_src;
    wire [1:0] result_src;
	wire mem_write;
    wire [2:0] alu_control;
    wire alu_src;
    wire [2:0] imm_src;
    wire reg_write;

    assign src_a = read_data_1;
    assign src_b = alu_src ? imm_ext : read_data_2;
    assign write_data_out = result_src == 2'b00 ? alu_result : (result_src == 2'b01 ? read_data : {{BUS_WIDTH - INST_MEMORY_ADDR_BUS_WIDTH{1'b0}}, pc_4});
    assign next_pc = pc_src ? pc_target[INST_MEMORY_ADDR_BUS_WIDTH - 1:0] : pc_4;

    // Instantiate control module
    control # (BUS_WIDTH) control_inst (
        .zero(zero),
        .instruction(instruction),
        .pc_src(pc_src),
        .result_src(result_src),
        .mem_write(mem_write),
        .alu_control(alu_control),
        .alu_src(alu_src),
        .imm_src(imm_src),
        .reg_write(reg_write)
    );

    // Instantiate adder for adding 4 to pc
    adder # (32) adder_inst1 (
        .a(pc),
        .b({{28{1'b0}}, 4'b0100}),
        .y(pc_4)
    );

    // Instantiate adder for adding pc and imm_ext
    adder # (BUS_WIDTH) adder_inst2 (
        .a({pc}),
        .b(imm_ext),
        .y(pc_target)
    );
   
    // Insntiate register_file module
    register_file #(REG_FILE_ADDR_BUS_WIDTH, REG_FILE_DATA_BUS_WIDTH) register_file_inst (
        .clk(clk),
        .addr1(instruction[19:15]),
        .addr2(instruction[24:20]),
        .addr3(instruction[11:7]),
        .write_data(write_data_out),
        .write_en(reg_write),
        .read_data1(read_data_1),
        .read_data2(read_data_2)
    );

    // Insntiate extend module
    extend #(BUS_WIDTH) extend_inst (
        .imm_src(imm_src),
        .instruction(instruction),
        .extended_imm(imm_ext)
    );

    // Insntiate alu module
    alu #(BUS_WIDTH) alu_inst (
        .src_a(src_a),
        .src_b(src_b),
        .alu_op(alu_control),
        .alu_result(alu_result),
        .zero(zero)
    );

endmodule

module alu
    #(
        parameter BUS_WIDTH = 32
    )
    (
        input [BUS_WIDTH - 1:0] src_a,
        input [BUS_WIDTH - 1:0] src_b,
        input [2:0] alu_op,
        output reg [BUS_WIDTH - 1:0] alu_result,
        output reg zero
    );

    always @(alu_op or src_a or src_b)
    begin
        case(alu_op)
            3'b000: alu_result = src_a + src_b;
            3'b001: begin
                alu_result = src_a - src_b;
                 if (src_a == src_b)
                     zero = 1'b1;
                 else
                     zero = 1'b0;
            end
            3'b010: alu_result = src_a | src_b;
            3'b011: alu_result = src_a & src_b;
            3'b101: begin
                if (src_a < src_b)
                    alu_result = {{BUS_WIDTH - 1{1'b0}}, 1'b1};
                else
                    alu_result = {BUS_WIDTH{1'b0}};
            end
            3'b110: alu_result = src_b;
            default: alu_result = 0;
        endcase
    end

     initial
     begin
         zero = 1'b0;
     end
endmodule

module control_alu_decoder
    (
        input [6:0] opcode,
        input funct7_5,
        input [2:0] funct3,
        input [1:0] alu_op,
        output reg [2:0] alu_control
    );

    always @(opcode, funct7_5, funct3, alu_op)
    begin
        case(alu_op)
            2'b00: alu_control = 3'b000;    // add
            2'b01: alu_control = 3'b001;    // sub
            2'b10: 
            begin
                case(funct3)
                    3'b000: 
                    begin
                        if({opcode[5], funct7_5} == 2'b11)
                        begin
                            alu_control = 3'b001;   // sub
                        end
                        else
                        begin
                            alu_control = 3'b000;   // add
                        end
                    end
                    3'b010: alu_control = 3'b101;  // slt (set less than)
                    3'b110: alu_control = 3'b010;   // or
                    3'b111: alu_control = 3'b011;   // and
                    default: alu_control = 3'bxxx;
                endcase
            end
            2'b11: alu_control = 3'b110;    // just pass
            default: alu_control = 3'bxxx;
        endcase
    end
endmodule

module control_main_decoder
    (
        input [6:0] opcode,
        output reg branch,
        output reg [1:0] result_src,
        output reg mem_write,
        output reg alu_src,
        output reg [2:0] imm_src,
        output reg reg_write,
        output reg [1:0] alu_op,
        output reg jump
    );

    always @(opcode)
    begin
        case(opcode)
            7'd3: begin //lw (I type)
                branch = 1'b0;
                result_src = 2'b01;
                mem_write = 1'b0;
                alu_src = 1'b1;
                imm_src = 3'b000;
                reg_write = 1'b1;
                alu_op = 2'b00;
                jump = 1'b0;
            end
            7'd35: begin    // sw (S type)
                branch = 1'b0;
                result_src = 2'bxx;
                mem_write = 1'b1;
                alu_src = 1'b1;
                imm_src = 3'b001;
                reg_write = 1'b0;
                alu_op = 2'b00;
                jump = 1'b0;
            end
            7'd51: begin // or (R type)
                branch = 1'b0;
                result_src = 2'b00;
                mem_write = 1'b0;
                alu_src = 1'b0;
                imm_src = 3'bxxx;
                reg_write = 1'b1;
                alu_op = 2'b10;
                jump = 1'b0;
            end
            7'd99: begin  // beq (B type)
                branch = 1'b1;
                result_src = 2'bxx;
                mem_write = 1'b0;
                alu_src = 1'b0;
                imm_src = 3'b010;
                reg_write = 1'b0;
                alu_op = 2'b01;
                jump = 1'b0;
            end
            7'd19: begin  // addi (I type)
                branch = 1'b0;
                result_src = 2'b00;
                mem_write = 1'b0;
                alu_src = 1'b1;
                imm_src = 3'b000;
                reg_write = 1'b1;
                alu_op = 2'b10;
                jump = 1'b0;
            end
            7'd111: begin    // jal (J type)
                branch = 1'b0;
                result_src = 2'b10;
                mem_write = 1'b0;
                alu_src = 1'bx;
                imm_src = 3'b011;
                reg_write = 1'b1;
                alu_op = 2'bxx;
                jump = 1'b1;
            end
            7'd55: begin    // lui (U type)
                branch = 1'b0;
                result_src = 2'b00;
                mem_write = 1'b0;
                alu_src = 1'b1;
                imm_src = 3'b100;
                reg_write = 1'b1;
                alu_op = 2'b11;
                jump = 1'b0;
            end
            default: begin
                branch = 1'bx;
                result_src = 2'bxx;
                mem_write = 1'bx;
                alu_src = 1'bx;
                imm_src = 3'bxxx;
                reg_write = 1'bx;
                alu_op = 2'bxx;
                jump = 1'b0;
            end
        endcase
    end
endmodule

module control
    #(parameter INSTR_WIDTH = 32)
    (
        input zero,
        input [INSTR_WIDTH - 1: 0] instruction,
        output pc_src,
        output [1:0] result_src,
        output mem_write,
        output [2:0] alu_control,
        output alu_src,
        output [2:0] imm_src,
        output reg_write
    );

    wire [6:0] opcode = instruction[6:0];
    wire [6:0] funct7 = instruction[31:25];
    wire [2:0] funct3 = instruction[14:12];

    wire [1:0] alu_op;

    wire branch;
    wire jump;

    control_main_decoder control_main_decoder_inst (
        .opcode(opcode),
        .branch(branch),
        .result_src(result_src),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .imm_src(imm_src),
        .reg_write(reg_write),
        .alu_op(alu_op),
        .jump(jump)
    );

    control_alu_decoder control_alu_decoder_inst (
        .opcode(opcode),
        .funct7_5(funct7[5]),
        .funct3(funct3),
        .alu_op(alu_op),
        .alu_control(alu_control)
    );

    reg pc_src_reg;

    always @(*) begin
        if (jump)
            pc_src_reg = 1'b1;
        else
            pc_src_reg = zero & branch;
    end
    
    assign pc_src = pc_src_reg;
endmodule

module extend
    #(parameter DATA_BUS_WIDTH = 32)
    (
        input [2:0] imm_src,
        input [DATA_BUS_WIDTH - 1:0] instruction,
        output reg [DATA_BUS_WIDTH - 1:0] extended_imm
    );

    always @(imm_src, instruction)
    begin
        case (imm_src)
            3'b000 : extended_imm = {{20{instruction[31]}}, instruction[31:20]}; // I-type
            3'b001 : extended_imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};  // S-type
            3'b010 : extended_imm = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type
            3'b011 : extended_imm = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};    // J-type
            3'b100 : extended_imm = {instruction[31:12], {12{1'b0}}};    // U-type
            default : extended_imm = 32'b0;
        endcase
    end
endmodule

module register_file
    #(
        parameter ADDR_BUS_WIDTH = 5,
        parameter DATA_BUS_WIDTH = 32
    )
    (
        input clk,
        input [ADDR_BUS_WIDTH - 1:0] addr1,
        input [ADDR_BUS_WIDTH - 1:0] addr2,
        input [ADDR_BUS_WIDTH - 1:0] addr3,
        input [DATA_BUS_WIDTH - 1:0] write_data,
        input write_en,
        output [DATA_BUS_WIDTH - 1:0] read_data1,
        output [DATA_BUS_WIDTH - 1:0] read_data2
    );

    localparam MEM_DEPTH = 32;

    reg [DATA_BUS_WIDTH - 1:0] mem [0:MEM_DEPTH - 1];
    
    // Register 0 is always 0.
    assign read_data1 = addr1 == 5'b00000 ? {32{1'b0}} : mem[addr1];
    assign read_data2 = addr2 == 5'b00000 ? {32{1'b0}} : mem[addr2];
	 
	initial
	begin
        mem[5] = 32'h00000006;
        mem[9] = 32'h00002004;
	end

    always @(negedge clk)
    begin
        if(write_en)
        begin
            mem[addr3] <= write_data;
        end
    end
endmodule

module adder
    # (parameter BUS_WIDTH = 16)
    (
    input wire [BUS_WIDTH - 1:0] a, b,
    output wire [BUS_WIDTH - 1:0] y
    );

    assign  y = a + b;
endmodule
