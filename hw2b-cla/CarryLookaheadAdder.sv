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

   wire g0out;
   wire p0out;
   wire [2:0] c0out;
   gp4 a(.gin(gin[3:0]), .pin(pin[3:0]), .cin(cin), .gout(g0out), .pout(p0out), .cout(c0out));

   wire c4 = g0out | (p0out & cin);

   wire g1out;
   wire p1out;
   wire [2:0] c1out;
   gp4 b(.gin(gin[7:4]), .pin(pin[7:4]), .cin(c4), .gout(g1out), .pout(p1out), .cout(c1out));

   assign gout = g1out | (p1out & g0out);
   assign pout = p1out & p0out;
   assign cout = {c1out, c4, c0out};


endmodule

module CarryLookaheadAdder
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

   wire [31:0] g;
   wire [31:0] p;
   genvar i;
   for (i = 0; i < 32; i = i + 1) begin
      gp1 gp(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
   end

   wire [3:0] g8out;
   wire [3:0] p8out;

   wire [6:0] c8_0;
   gp8 gp_0(.gin(g[7:0]), .pin(p[7:0]), .cin(cin), .gout(g8out[0]), .pout(p8out[0]), .cout(c8_0));
   wire cin_1 = g8out[0] | (p8out[0] & cin);

   wire [6:0] c8_1;
   gp8 gp_1(.gin(g[15:8]), .pin(p[15:8]), .cin(cin_1), .gout(g8out[1]), .pout(p8out[1]), .cout(c8_1));
   wire cin_2 = g8out[1] | (p8out[1] & cin_1);

   wire [6:0] c8_2;
   gp8 gp_2(.gin(g[23:16]), .pin(p[23:16]), .cin(cin_2), .gout(g8out[2]), .pout(p8out[2]), .cout(c8_2));
   wire cin_3 = g8out[2] | (p8out[2] & cin_2);

   wire [6:0] c8_3;
   gp8 gp_3(.gin(g[31:24]), .pin(p[31:24]), .cin(cin_3), .gout(g8out[3]), .pout(p8out[3]), .cout(c8_3));

   wire [31:0] c = {c8_3, cin_3, c8_2, cin_2, c8_1, cin_1, c8_0, cin};

   assign sum = a ^ b ^ c;

endmodule
