/*
* TCAS Simulator
* ECEN 603 Winter 2025
* By: Clayton Wiley & Samuel Stephen
* This module simulates the Traffic Collision Avoidance System (TCAS) for an aircraft.
* It takes in the aircraft's position and velocity data, and outputs the TCAS resolution and level.
* The TCAS resolution indicates the action to be taken (e.g., climb, descend, or maintain altitude),
* while the TCAS level indicates the severity of the situation (e.g., normal, caution, or warning).
*/

module tcas_simulator(
    input clk, fly,                    // Clock & Enable Bit
    input [15:0] px, py, pz,           // Position Vector
    input [15:0] vx, vy, vz,           // Velocity Vector
    output reg [2:0] tcas_resolution,  // 
    output reg [2:0] tcas_level
);

// +==========[ STATE CONTROL ]==========+
reg [1:0] CS, NS; 

// +==========[ STATES ]==========+
parameter STARTUP    = 2'b00,
          ALL_CLEAR  = 2'b01,
          TRAFFIC    = 2'b10,
          RESOLUTION = 2'b11;

// +==========[ TCAS LEVELS ]==========+
parameter LEVEL_RESET = 2'b00,
          LEVEL_NONE  = 2'b01,
          LEVEL_1     = 2'b10, 
          LEVEL_2     = 2'b11;

// +==========[ TCAS RESOLUTIONS ]==========+
parameter RESOLUTION_RESET   = 3'b000,
          RESOLUTION_NONE    = 3'b001,
          RESOLUTION_DESCEND = 3'b010,
          RESOLUTION_CLIMB   = 3'b011,
          RESOLUTION_ERROR   = 3'b100,

// +==========[ TCAS THRESHOLDS ]==========+
parameter LEVEL_1_THRESHOLD = 2'b01,
          LEVEL_2_THRESHOLD = 2'b10;

function [2:0] threat_detection;
    input [15:0] px, py, pz;
    input [15:0] vx, vy, vz;
    begin
        // Placeholder
    end
endfunction

function advise_maneuver();
    input [15:0] px, py, pz;
    input [15:0] vx, vy, vz;
    begin
        // Placeholder
    end
endfunction

always @(posedge clk or posedge fly)
begin
    if(fly == 0)
    begin
        CS <= STARTUP;
    end
    else
    begin
        CS <= NS;
    end
end

always @(*)
begin
    case(CS)
        STARTUP: 
        begin 
            if(fly == 1)
                NS = ALL_CLEAR; // Move to all clear state
            else
                NS = STARTUP; // Stay in startup state
        end
        ALL_CLEAR: 
        begin
            if(threat_detection(px, py, pz, vx, vy, vz) > LEVEL_1_THRESHOLD)
                NS = TRAFFIC;
            else
                NS = ALL_CLEAR;
        end
        TRAFFIC:
        begin
            if(threat_detection(px, py, pz, vx, vy, vz) > LEVEL_2_THRESHOLD)
                NS = RESOLUTION;
            else if(threat_detection(px, py, pz, vx, vy, vz) > LEVEL_1_THRESHOLD)
                NS = TRAFFIC;
            else
                NS = ALL_CLEAR;      
        end
        RESOLUTION:
        begin
            NS = TRAFFIC;
        end
        default: NS = STARTUP;
    endcase
end

always @(posedge clk)
begin
    case(CS)
        STARTUP: 
        begin
            tcas_level = LEVEL_RESET;
            tcas_resolution = RESOLUTION_RESET;
        end
        ALL_CLEAR:
        begin
            tcas_level = LEVEL_NONE;
            tcas_resolution = RESOLUTION_NONE;
        end
        TRAFFIC:
        begin
            if(threat_detection(px, py, pz, vx, vy, vz) > LEVEL_2_THRESHOLD)
                tcas_level = LEVEL_2;
            else if(threat_detection(px, py, pz, vx, vy, vz) > LEVEL_1_THRESHOLD)
                tcas_level = LEVEL_1;
            else
                ;
        end
        RESOLUTION:
        begin
            tcas_resolution = advise_maneuver(px, py, pz, vx, vy, vz);

            if(tcas_resolution > RESOLUTION_CLIMB && tcas_resolution < RESOLUTION_DESCEND)
                tcas_resolution = RESOLUTION_ERROR;
            else
                ;
        end
        default: ;
    endcase
end

endmodule