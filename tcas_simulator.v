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
    input clk, om,                     // Clock & Operation Mode Bit
    input [15:0] p1x, p1y, p1z,        // Position 1 Vector
    input [15:0] v1x, v1y, v1z,        // Velocity 1 Vector
    input [31:0] p2x, p2y, p2z,        // Position 2 Vector
    input [31:0] v2x, v2y, v2z,        // Velocity 2 Vector
    output reg [2:0] tcas_resolution,  // Advise Maneuver Output
    output reg [2:0] tcas_traffic      // Alerts if Advisory
);

// +==========[ STATE CONTROL ]==========+
reg [1:0] CS, NS;

// +==========[ STATES ]==========+
parameter STARTUP   = 2'b00,
          STANDBY   = 2'b01,
          OPERATING = 2'b10;

// +==========[ TCAS LEVELS ]==========+
parameter TRAFFIC_RESET     = 2'b00,
          TRAFFIC_NONE      = 2'b01,
          TRAFFIC_ADVISORY  = 2'b10;

// +==========[ TCAS Operation Modes ]==========+
parameter OM_RESET   = 2'b00,
          OM_TA_ONLY = 2'b01,
          OM_TA_RA   = 2'b10;
        
// +==========[ TCAS RESOLUTIONS ]==========+
parameter RA_RESET   = 3'b000,
          RA_NONE    = 3'b001,
          RA_DESCEND = 3'b010,
          RA_CLIMB   = 3'b011;

// +==========[ MATH CONSTANTS ]==========+
integer vmax = 7, amax = 9.8*0.25, react = 5;

reg [2:0] ta, ra; // Advisory Flags

always @(posedge clk or om)
begin
    if(om == OM_RESET)
        CS <= STARTUP;
    else
        CS <= NS;
end

always @(*)
begin
    case(CS)
        STARTUP: 
        begin 
            if(om != OM_RESET)
                NS = STANDBY;
            else
                NS = STARTUP; // Stay in startup state
        end
        STANDBY: 
        begin
            if(om == OM_TA_ONLY || om == OM_TA_RA)
                NS = OPERATING;
            else
                NS = STANDBY;
        end
        OPERATING:
        begin
            if(om == OM_TA_RA)
                NS = OPERATING;
            else if(om == OM_TA_ONLY)
                NS = OPERATING;
            else
                NS = STANDBY;
        end
        default: NS = STARTUP;
    endcase
end

always @(posedge clk)
begin
    case(CS)
        STARTUP: 
        begin
            tcas_traffic <= TRAFFIC_RESET;
            tcas_resolution <= RA_RESET;
        end
        STANDBY:
        begin
            ta <= 0;
            ra <= 0;
        end
        OPERATING:
        begin
            tcas_traffic <= ta;
            tcas_resolution <= ra;

            if(om == OM_TA_RA)
            begin
                ;
                // threat_detection(p1x, p1y, p1z, p2x, p2y, p2z, v1x, v1y, v1z, v2x, v2y, v2z, <missing vars>);
                // advise_maneuver(p1x, p1y, p1z, p2x, p2y, p2z, v1x, v1y, v1z, v2x, v2y, v2z, vmax, amax, react);
            end
            else if(om == OM_TA_ONLY)
            begin
                ;
                // threat_detection(p1x, p1y, p1z, p2x, p2y, p2z, v1x, v1y, v1z, v2x, v2y, v2z, <missing vars>);
            end
            else
            ;
        end
        default: ;
    endcase
end

endmodule

module advise_maneuver(
    input wire signed [31:0] p1_x, p1_y, p1_z,
    input wire signed [31:0] p2_x, p2_y, p2_z,
    input wire signed [31:0] v1_x, v1_y, v1_z,
    input wire signed [31:0] v2_x, v2_y, v2_z,
    input wire signed [31:0] vmax, amax, react,
    output reg [2:0] maneuver
);

// +==========[ TCAS RESOLUTIONS ]==========+
parameter RA_RESET   = 3'b000,
          RA_NONE    = 3'b001,
          RA_DESCEND = 3'b010,
          RA_CLIMB   = 3'b011;
          
    integer pr_x, pr_y, pr_z;
    integer vr_x, vr_y, vr_z;
    integer reusedValue, upDeviation, downDeviation;
    integer upDifference, downDifference;
    integer tca;

    always @(*) begin
        // Calculate relative vectors
        pr_x = p1_x - p2_x;
        pr_y = p1_y - p2_y;
        pr_z = p1_z - p2_z;

        vr_x = v1_x - v2_x;
        vr_y = v1_y - v2_y;
        vr_z = v1_z - v2_z;

        // Calculate time of closest approach
        tca = -(pr_x * vr_x + pr_y * vr_y + pr_z * vr_z) / (vr_x * vr_x + vr_y * vr_y + vr_z * vr_z);

        // Calculate maximum vertical deviation before TCA
        reusedValue = (v1_z * v1_z - vmax * vmax) / (2 * amax);
        upDeviation = vmax * (tca - react - (-v1_z + vmax) / amax) + (-reusedValue + react * v1_z);
        downDeviation = -vmax * (tca - react - (v1_z + vmax) / amax) + (reusedValue + react * v1_z);

        // Calculate the final vertical deviations
        upDifference = (p1_z + upDeviation - p2_z + tca * v2_z);
        downDifference = (p1_z + downDeviation - p2_z + tca * v2_z);

        // abs equivalent
        if(upDifference < 0) upDifference = upDifference*-1
        if(downDifference < 0) downDifference = downDifference*-1

        // Determine the maneuver (climb or descend)
        if (upDifference > downDifference) begin
            maneuver = RA_CLIMB;  // Climb
        end else begin
            maneuver = RA_DESCEND;  // Descend
        end
    end
endmodule

module threat_detection(
    input wire signed [31:0] p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, v1_x, v1_y, v1_z, v2_x, v2_y, v2_z,
    input wire signed [31:0] taurange_ta_th, tauvert_ta_th, range_ta_th, vert_ta_th,
    input wire signed [31:0] taurange_ra_th, tauvert_ra_th, range_ra_th, vert_ra_th,
    output reg [2:0] ta,  // Threat Advisory (TA)
    output reg [2:0] ra   // Resolution Advisory (RA)
);
    integer prmagsq, vrproj, taurange, tauvert, pr_x, pr_y, pr_z, vr_x, vr_y, vr_z;

    always @(*) begin
        // Calculate relative vectors
        pr_x = p1_x - p2_x;
        pr_y = p1_y - p2_y;
        pr_z = p1_z - p2_z;

        vr_x = v1_x - v2_x;
        vr_y = v1_y - v2_y;
        vr_z = v1_z - v2_z;

        // Calculate magnitude squared of pr
        prmagsq = pr_x * pr_x + pr_y * pr_y + pr_z * pr_z;

        // Calculate the projection of pr on vr
        vrproj = pr_x * vr_x + pr_y * vr_y + pr_z * vr_z;

        // Calculate time to closest range approach 
        taurange = prmagsq / vrproj;

        // Calculate time to vertical range approach
        tauvert = pr_z / vr_z;

        // Default to no threat
        ta = 0;
        ra = 0;

        // Check if the conditions for TA (Threat Advisory) are met
        if (taurange < taurange_ta_th && tauvert < tauvert_ta_th) begin
            if (prmagsq < range_ta_th && pr_z < vert_ta_th) begin
                ta = 1;
            end
        end

        // Check if the conditions for RA (Resolution Advisory) are met
        if (taurange < taurange_ra_th && tauvert < tauvert_ra_th) begin
            if (prmagsq < range_ra_th && pr_z < vert_ra_th) begin
                ra = 1;
            end
        end
    end
endmodule
