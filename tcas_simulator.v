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
    input clk, 
    input [1:0] om,                                 // Clock & Operation Mode Bit
    input wire signed [15:0] p1x, p1y, p1z,        // Position 1 Vector
    input wire signed [15:0] v1x, v1y, v1z,        // Velocity 1 Vector
    input wire signed [31:0] p2x, p2y, p2z,        // Position 2 Vector
    input wire signed [31:0] v2x, v2y, v2z,        // Velocity 2 Vector
    input wire signed [31:0] altitude,             // "Aircraft" altitude
    output reg [2:0] tcas_resolution,              // Advise Maneuver Output
    output reg [2:0] tcas_traffic                  // Alerts if Advisory
);

task thresholds(
    input [31:0] altitude,         // 32-bit input representing the altitude
    output reg [31:0] tau_ta_th,   // Output tau_ta_th
    output reg [31:0] range_ta_th, // Output range_ta_th
    output reg [31:0] vert_ta_th,  // Output vert_ta_th
    output reg [31:0] tau_ra_th,   // Output tau_ra_th
    output reg [31:0] range_ra_th, // Output range_ra_th
    output reg [31:0] vert_ra_th   // Output vert_ra_th
);
    integer sl;

    begin
        if (altitude < 3280) begin
            sl = 2;
        end else if (altitude < 7710) begin
            sl = 3;
        end else if (altitude < 16400) begin
            sl = 4;
        end else if (altitude < 32800) begin
            sl = 5;
        end else if (altitude < 65600) begin
            sl = 6;
        end else if (altitude < 137760) begin
            sl = 7;
        end else begin
            sl = 8;
        end
        // $display("sl = %d", sl);
        // Select the threshold values based on 'sl'
        case (sl)
            2: begin
                tau_ta_th = 400;
                range_ta_th = 560;
                vert_ta_th = 260;
                tau_ra_th = 0;
                range_ra_th = 0;
                vert_ra_th = 0;
            end
            3: begin
                tau_ta_th = 625;
                range_ta_th = 615;
                vert_ta_th = 260;
                tau_ra_th = 225;
                range_ra_th = 375;
                vert_ra_th = 185;
            end
            4: begin
                tau_ta_th = 900;
                range_ta_th = 890;
                vert_ta_th = 260;
                tau_ra_th = 400;
                range_ra_th = 650;
                vert_ra_th = 185;
            end
            5: begin
                tau_ta_th = 1600;
                range_ta_th = 1390;
                vert_ta_th = 260;
                tau_ra_th = 625;
                range_ra_th = 1020;
                vert_ra_th = 185;
            end
            6: begin
                tau_ta_th = 2025;
                range_ta_th = 1855;
                vert_ta_th = 260;
                tau_ra_th = 900;
                range_ra_th = 1485;
                vert_ra_th = 185;
            end
            7: begin
                tau_ta_th = 2304;
                range_ta_th = 2410;
                vert_ta_th = 260;
                tau_ra_th = 1225;
                range_ra_th = 2040;
                vert_ra_th = 215;
            end
            8: begin
                tau_ta_th = 2304;
                range_ta_th = 2410;
                vert_ta_th = 365;
                tau_ra_th = 1225;
                range_ra_th = 2040;
                vert_ra_th = 250;
            end
            default: begin
                tau_ta_th = 0;
                range_ta_th = 0;
                vert_ta_th = 0;
                tau_ra_th = 0;
                range_ra_th = 0;
                vert_ra_th = 0;
            end
        endcase
    end
endtask

task advise_maneuver(
    input signed [31:0] p1_x, p1_y, p1_z,
    input signed [31:0] p2_x, p2_y, p2_z,
    input signed [31:0] v1_x, v1_y, v1_z,
    input signed [31:0] v2_x, v2_y, v2_z,
    input signed [31:0] vmax, amax, react,
    output reg [2:0] maneuver
);
// +==========[ TCAS RESOLUTIONS ]==========+
parameter RA_NONE    = 3'b000,
          RA_TRIGGER = 3'b001,
          RA_DESCEND = 3'b010,
          RA_CLIMB   = 3'b011;
          
    integer pr_x, pr_y, pr_z;
    integer vr_x, vr_y, vr_z;
    integer reusedValue, upDeviation, downDeviation;
    integer upDifference, downDifference;
    integer tca;

    // always @(*) begin
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
    reusedValue = ((v1_z * v1_z) - (vmax * vmax)) / (2 * amax);
    downDeviation = -vmax*(tca - react - (v1_z + vmax)/amax) + (reusedValue + react*v1_z);
    upDeviation = vmax*(tca - react - (-v1_z + vmax)/amax) + (-reusedValue + react*v1_z);

    // Calculate the final vertical deviations
    upDifference = (p1_z + upDeviation - p2_z + tca * v2_z);
    downDifference = (p1_z + downDeviation - p2_z + tca * v2_z);

    // abs equivalent
    upDifference = upDifference > 0 ? upDifference : upDifference*-1;
    downDifference = downDifference > 0 ? downDifference : downDifference*-1;

    // Determine the maneuver (climb or descend)
    if (upDifference > downDifference) begin
        maneuver = RA_CLIMB;  // Climb
    end else
        maneuver = RA_DESCEND;  // Descend

endtask

task threat_detection(
    input signed [31:0] p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, v1_x, v1_y, v1_z, v2_x, v2_y, v2_z,
    input signed [31:0] altitude,
    output reg [2:0] ta,  // Threat Advisory (TA)
    output reg [2:0] ra   // Resolution Advisory (RA)
);
    integer prmagsq, vrmagsq, vrproj, taurange, tauvert, pr_x, pr_y, pr_z, vr_x, vr_y, vr_z;
    integer tau_ta_th, range_ta_th, vert_ta_th, tau_ra_th, range_ra_th, vert_ra_th;

    // +==========[ TCAS LEVELS ]==========+
    parameter TRAFFIC_NONE = 2'b00,
          TRAFFIC_ADVISORY = 2'b01;

// +==========[ TCAS RESOLUTIONS ]==========+
    parameter RA_NONE = 3'b000,
          RA_TRIGGER  = 3'b001,
          RA_DESCEND  = 3'b010,
          RA_CLIMB    = 3'b011;

    // always @(*)
    begin
        // Calculate relative vectors
        pr_x = p1_x - p2_x;
        pr_y = p1_y - p2_y;
        pr_z = p1_z - p2_z;

        // $display("pr_x = %d, pr_y = %d, pr_z = %d", pr_x, pr_y, pr_z);

        vr_x = v1_x - v2_x;
        vr_y = v1_y - v2_y;
        vr_z = v1_z - v2_z;

        // Calculate thresholds based on altitude (p1_z)
        thresholds (altitude, tau_ta_th, range_ta_th, vert_ta_th, tau_ra_th, range_ra_th, vert_ra_th);

        // Calculate magnitude squared of pr
        prmagsq = pr_x * pr_x + pr_y * pr_y + pr_z * pr_z;
        // $display("prmagsq = %d", prmagsq);

        // Calculate magnitude squared of vr
        vrmagsq = vr_x * vr_x + vr_y * vr_y + vr_z * vr_z;

        // Calculate the projection of pr on vr
        vrproj = pr_x * vr_x + pr_y * vr_y + pr_z * vr_z;
        // $display("vrproj = %d", vrproj);

        // Calculate time to closest range approach
        if(vrproj == 0)
            taurange = 500;
        else
            taurange = prmagsq / vrmagsq;

        // $display("taurange = %d", taurange);

        if(vr_z == 0)
            tauvert = 500;
        else
            tauvert = (pr_z * pr_z) / (vr_z * vr_z);

        // $display("tauvert = %d", tauvert);

        // Default to no threat
        ta = TRAFFIC_NONE;
        ra = RA_NONE;

        // Check if the conditions for TA (Threat Advisory) are met
        if ((taurange < tau_ta_th || prmagsq < range_ta_th * range_ta_th) && (tauvert < tau_ta_th || pr_z < vert_ta_th))
        begin
            // $display("CONDITION PASSED FOR TA WOOO");
            ta = TRAFFIC_ADVISORY;
        end
        else
            ;
            // $display("CONDITION FAILED, %b", taurange < tau_ra_th);

        // Check if the conditions for RA (Resolution Advisory) are met
        if ((taurange < tau_ra_th || prmagsq < range_ra_th * range_ra_th) && (tauvert < tau_ra_th || pr_z < vert_ra_th))
            ra = RA_TRIGGER;
        else
            ;
    end
endtask

// +==========[ STATE CONTROL ]==========+
reg [1:0] CS = 0, NS = 0;

// +==========[ STATES ]==========+
parameter STARTUP   = 2'b00,
          STANDBY   = 2'b01,
          OPERATING = 2'b10;

// +==========[ TCAS LEVELS ]==========+
parameter TRAFFIC_NONE     = 2'b00,
          TRAFFIC_ADVISORY = 2'b01;

// +==========[ TCAS Operation Modes ]==========+
parameter OM_RESET   = 2'b00,
          OM_TA_ONLY = 2'b01,
          OM_TA_RA   = 2'b10;

// +==========[ TCAS RESOLUTIONS ]==========+
parameter RA_NONE    = 3'b000,
          RA_TRIGGER = 3'b001,
          RA_DESCEND = 3'b010,
          RA_CLIMB   = 3'b011;

// +==========[ MATH CONSTANTS ]==========+
integer vmax = 7, amax = 9.8*0.25, react = 5;

reg [2:0] ta, ra; // Advisory Flags

always @(posedge clk or negedge om)
begin
    // $display("Current State: %b, OM: %b", CS, om);
    if(om == OM_RESET) begin
        CS <= STARTUP; end
    else begin
        CS <= NS; end
end

always @(*)
begin
    case(CS)
        STARTUP: 
        begin 
            if(om != OM_RESET)
                NS <= STANDBY;
            else
                NS <= STARTUP; // Stay in startup state
        end
        STANDBY:
        begin
            if(om == OM_TA_ONLY || om == OM_TA_RA)
                NS <= OPERATING;
            else
                NS <= STANDBY;
        end
        OPERATING:
        begin
            if(om == OM_TA_RA)
                NS <= OPERATING;
            else if(om == OM_TA_ONLY)
                NS <= OPERATING;
            else
                NS <= STANDBY;
        end
        default: NS <= STARTUP;
    endcase
end

always @(posedge clk)
begin
    case(CS)
        STARTUP: 
        begin
            tcas_traffic <= TRAFFIC_NONE;
            tcas_resolution <= RA_NONE;
        end
        STANDBY:
        begin
            ta = 0;
            ra = 0;
        end
        OPERATING:
        begin
            tcas_traffic <= ta;
            tcas_resolution <= ra;

            if(om == OM_TA_RA)
            begin
                threat_detection (p1x, p1y, p1z, p2x, p2y, p2z, v1x, v1y, v1z, v2x, v2y, v2z, altitude, ta, ra);
                if(ra == RA_TRIGGER)
                    advise_maneuver (p1x, p1y, p1z, p2x, p2y, p2z, v1x, v1y, v1z, v2x, v2y, v2z, vmax, amax, react, ra);
            end
            else if(om == OM_TA_ONLY)
            begin
                threat_detection (p1x, p1y, p1z, p2x, p2y, p2z, v1x, v1y, v1z, v2x, v2y, v2z, altitude, ta, ra);
                // $display("Hello ta = %d, ra = %d", ta, ra);
            end
            else
            ;
        end
        default: ;
    endcase
end

endmodule


