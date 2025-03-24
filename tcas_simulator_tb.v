module tcas_simulator_tb();

reg clk;
reg [1:0] om;
reg signed [15:0] px_1, py_1, pz_1, vx_1, vy_1, vz_1;
reg signed [15:0] px_2, py_2, pz_2, vx_2, vy_2, vz_2;
reg [31:0] altitude;

wire [2:0] resolution_advisory;
wire [2:0] traffic_advisory;

tcas_simulator dut (clk, om, px_1, py_1, pz_1, vx_1, vy_1, vz_1, px_2, py_2, pz_2, vx_2, vy_2, vz_2, altitude, resolution_advisory, traffic_advisory);

initial begin
    clk = 0;
end

always begin
    #10 clk = ~clk;
end

initial begin
    $display("+=========[RESET]=========+");
    om = 2'b00;
    // Airplane 1
    px_1 = 0; py_1 = 0; pz_1 = 0;
    vx_1 = 0; vy_1 = 0; vz_1 = 0;
    altitude = 0;

    // Airplane 2
    px_2 = 0; py_2 = 0; pz_2 = 0;
    vx_2 = 0; vy_2 = 0; vz_2 = 0;

    #100 $display("+=========[TEST 1]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -6000; pz_1 = 15000;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -6000; py_2 = 0; pz_2 = 15000;
    vx_2 = 150; vy_2 = 0; vz_2 = 0;

    om = 2'b10; // Set the operation mode
    $display("No change so monitor won't print.");

    #100 $display("+=========[TEST 2]=========+");
    // Airplane 1
    
    px_1 = 0; py_1 = -4000; pz_1 = 15000;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -4000; py_2 = 0; pz_2 = 15000;
    vx_2 = 150; vy_2 = 0; vz_2 = 0;

    #100 $display("+=========[TEST 3]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -2500; pz_1 = 15000;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -2500; py_2 = 0; pz_2 = 15000;
    vx_2 = 150; vy_2 = 0; vz_2 = 0;

    #100 $display("+=========[TEST 4]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -2500; pz_1 = 15100;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -2500; py_2 = 0; pz_2 = 15000;
    vx_2 = 150; vy_2 = 0; vz_2 = 0;

    #100 $display("+=========[TEST 5]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -2500; pz_1 = 14900;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -2500; py_2 = 0; pz_2 = 15000;
    vx_2 = 150; vy_2 = 0; vz_2 = 0;

    #100 $display("+=========[TEST 6]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -6000; pz_1 = 15000;
    vx_1 = 0; vy_1 = 400; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -6000; py_2 = 0; pz_2 = 15000;
    vx_2 = 400; vy_2 = 0; vz_2 = 0;

    $display("No change so monitor won't print.");

    #100 $display("+=========[TEST 7]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -8000; pz_1 = 15000;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = -600; py_2 = -8000; pz_2 = 15000;
    vx_2 = 15; vy_2 = 150; vz_2 = 0;

    $display("No change so monitor won't print.");

    #100 $display("+=========[TEST 8]=========+");
    // Airplane 1
    px_1 = 0; py_1 = -8000; pz_1 = 15000;
    vx_1 = 0; vy_1 = 150; vz_1 = 0;
    altitude = pz_1;

    // Airplane 2
    px_2 = 0; py_2 = -8000; pz_2 = 14900;
    vx_2 = 0; vy_2 = 150; vz_2 = 2;

    #200 $finish;
end

initial begin
    $monitor("traffic_advisory (TA) = %d, resolution_advisory (RA) = %d @ t = ", traffic_advisory, resolution_advisory, $time);
end

endmodule