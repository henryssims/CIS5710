/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module DividerUnsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    // TODO: your code here
    wire [31:0] dividends[33];
    wire [31:0] remainders[33];
    wire [31:0] quotients[33];
    assign dividends[0] = i_dividend;
    assign quotients[0] = 32'b0;
    assign remainders[0] = 32'b0;
    genvar i;
    for (i = 0; i < 32; i = i+1) begin
        DividerOneIter d(.i_dividend(dividends[i]), .i_divisor(i_divisor), .i_remainder(remainders[i]), .i_quotient(quotients[i]), 
        .o_dividend(dividends[i+1]), .o_remainder(remainders[i+1]), .o_quotient(quotients[i+1]));
    end
    assign o_remainder = remainders[32];
    assign o_quotient = quotients[32];
endmodule


module DividerOneIter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
  /*
    for (int i = 0; i < 32; i++) {
        remainder = (remainder << 1) | ((dividend >> 31) & 0x1);
        if (remainder < divisor) {
            quotient = (quotient << 1);
        } else {
            quotient = (quotient << 1) | 0x1;
            remainder = remainder - divisor;
        }
        dividend = dividend << 1;
    }
    */

    // TODO: your code here
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
