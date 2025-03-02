module core_tb();

    reg clk;
    reg rst;
    reg [31:0] instruction;
    reg [31:0] pc;
    wire [31:0] memory_address;
    wire [31:0] data_to_write;
    wire [2:0] func3;
    wire write_data;
    reg [31:0] read_data;
    wire [31:0] next_pc;

    // Instantiate the Device Under Test (DUT)
    core dut (
        .clk(clk),
        .rst(rst),
        .instruction(instruction),
        .pc(pc),
        .memory_address(memory_address),
        .data_to_write(data_to_write),
        .func3(func3),
        .write_data(write_data),
        .read_data(read_data),
        .next_pc(next_pc)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test sequence
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, dut);

        // Initialize inputs
        rst = 1;
        instruction = 32'h00000000;
        pc = 32'h00000000;
        read_data = 32'h00000000;

        // Apply reset
        #10;
        rst = 0;

        // Test case 1: Add instruction
        instruction = 32'h002081B3; // add x3, x1, x2
        pc = 32'h00000004;
        read_data = 32'h00000000;
        #10;

        // Test case 2: Load instruction
        instruction = 32'h00012283; // lw x5, 0(x2)
        pc = 32'h00000008;
        read_data = 32'h00000006;
        #10;

        // Test case 3: Store instruction
        instruction = 32'h00412023; // sw x4, 0(x2)
        pc = 32'h0000000C;
        read_data = 32'h00000000;
        #10;

        // Test case 4: Branch instruction
        instruction = 32'h00018063; // beq x3, x0, 8
        pc = 32'h00000010;
        read_data = 32'h00000000;
        #10;

        // Test case 5: Jump instruction
        instruction = 32'h0000006F; // jal x0, 0
        pc = 32'h00000014;
        read_data = 32'h00000000;
        #10;

        // Finish simulation
        $finish();
    end

endmodule