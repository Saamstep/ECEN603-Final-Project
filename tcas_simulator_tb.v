module tcas_simulator_tb();

reg clk, fly;
reg [15:0] px, py, pz, vx, vy, vz;

wire [2:0] tcas_resolution;
wire [2:0] tcas_level;

always begin
    #5 clk = ~clk;
end

initial begin
    $display("+=========[Test Case 1]=========+");





    #200 $finish
end

endmodule