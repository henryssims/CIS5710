`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

   // TODO: your code here
   assign gout = (gin[3]) | (pin[3] & gin[2]) | (& pin[3:2] & gin[1]) | (& pin[3:1] & gin[0]);
   assign pout = (& pin);
   wire c1 = gin[0] | (pin[0] & cin);
   wire c2 = gin[1] | (pin[1] & c1);
   wire c3 = gin[2] | (pin[2] & c2);
   assign cout = {c3, c2, c1};


endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   // TODO: your code here
   wire g0out;
   wire p0out;
   wire [2:0] c0out;
   gp4 a(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin), .gout(g0out), .pout(p0out), .cout(c0out));

   wire g1out;
   wire p1out;
   wire [2:0] c1out;
   gp4 a(.gin(gin[7:4]), .pin(pin[7:4]), .cin(cin), .gout(g1out), .pout(p1out), .cout(c1out));

   assign gout = g1out | (p1out & g0out);
   assign pout = p1out & p0out;
   assign cout = {c1out, c0out};


endmodule

module CarryLookaheadAdder
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);
   wire [31:0] g;
   wire [31:0] p;
   genvar i;
   for (i = 0; i < 32; i = i + 1) begin
      gp1 g(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
   end

   wire [7:0] gout;
   wire [7:0] pout;
   wire [31:0] c;
   assign c[0] = cin;
   genvar j;
   for (j = 0; j < 8; j = j + 1) begin
      gp4 h(.gin(g[j*4 + 3: j*4]), .pin(p[j*4 + 3: j*4], .cin(c[j*4])), .gout(gout[j]), .pout(pout[j]), .cout(c[j*4 + 3: j*4 + 1]));
      assign c[j*4 + 4] = gout[j] | (pout[j] & c[j*4]);
   end

   wire [3:0] g8out;
   wire [3:0] p8out;
   wire [31:0] c8;
   assign c8[0] = cin;
   genvar k;
   for (k = 0; k < 4; k = k + 1) begin
      gp8 e(.gin(g[k*8 + 7: k*8], .pin(p[k*8 + 7: k*8]), .cin(c8[k*8]), .gout(g8out[k]), .pout(p8out[k]), .cout(c8[k*8 + 7: k*8 + 1])));
      assign c8[k*8 + 8] = g8out[k] | (pout[k] & c8[k*8]);
   end

   // TODO: your code here

endmodule
