/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module DividerUnsignedPipelined (
    input wire clk, rst, stall,
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);
    logic [31:0] dividend_pipe[0:7];
    logic [31:0] remainder_pipe[0:7];
    logic [31:0] quotient_pipe[0:7];
    logic [31:0] divisor_pipe[0:7];

    assign dividend_pipe[0] = i_dividend;
    assign remainder_pipe[0] = 32'b0;
    assign quotient_pipe[0] = 32'b0;
    assign divisor_pipe[0] = i_divisor;

    genvar i;
    for (i = 0; i < 7; i=i+1) begin

        logic [31:0] d1, r1, q1;
        logic [31:0] d2, r2, q2;
        logic [31:0] d3, r3, q3;
        logic [31:0] d4, r4, q4;

        divu_1iter iter0(dividend_pipe[i], divisor_pipe[i], remainder_pipe[i], quotient_pipe[i], d1, r1, q1);

        divu_1iter iter1(d1, divisor_pipe[i], r1, q1, d2, r2, q2);

        divu_1iter iter2(d2, divisor_pipe[i], r2, q2, d3, r3, q3);

        divu_1iter iter3(d3, divisor_pipe[i], r3, q3, d4, r4, q4);

        always_ff @(posedge clk) begin
            if (rst) begin
                dividend_pipe[i+1]  <= 32'b0;
                remainder_pipe[i+1] <= 32'b0;
                quotient_pipe[i+1]  <= 32'b0;
                divisor_pipe[i+1]   <= 32'b0;
            end else begin
                dividend_pipe[i+1]  <= d4;
                remainder_pipe[i+1] <= r4;
                quotient_pipe[i+1]  <= q4;
                divisor_pipe[i+1]   <= divisor_pipe[i];
            end
        end
    end

    logic [31:0] d1, r1, q1;
    logic [31:0] d2, r2, q2;
    logic [31:0] d3, r3, q3;
    logic [31:0] d4, r4, q4;

    divu_1iter iter0(dividend_pipe[7], divisor_pipe[7],
                        remainder_pipe[7], quotient_pipe[7],
                        d1, r1, q1);

    divu_1iter iter1(d1, divisor_pipe[7], r1, q1,
                        d2, r2, q2);

    divu_1iter iter2(d2, divisor_pipe[7], r2, q2,
                        d3, r3, q3);

    divu_1iter iter3(d3, divisor_pipe[7], r3, q3,
                        d4, r4, q4);

    assign o_remainder = r4;
    assign o_quotient  = q4;

endmodule

module divu_1iter (
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    input  wire  [31:0] i_remainder,
    input  wire  [31:0] i_quotient,
    output logic [31:0] o_dividend,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

    // TODO: copy your code from HW2A here
    wire [31:0] remainder_new = (i_remainder << 1) | ((i_dividend >> 31) & 32'b1);
    logic [31:0] quotient, remainder_subtracted;
    always_comb begin
        quotient = i_quotient << 1;
        remainder_subtracted = remainder_new;
        if (remainder_new >= i_divisor) begin
            quotient = quotient | 32'b1;
            remainder_subtracted = remainder_subtracted - i_divisor;
        end
    end
    assign o_dividend = i_dividend << 1;
    assign o_remainder = remainder_subtracted;
    assign o_quotient = quotient;
endmodule
