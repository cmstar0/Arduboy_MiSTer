/**
Descripcion,
Modulo que sincroniza las senales (hsync y vsync)
de un controlador VGA de 640x480 60hz, funciona con un reloj de 25Mhz

Ademas tiene las coordenadas de los pixeles H (eje x),
y de los pixeles V (eje y). Para enviar la senal RGB correspondiente
a cada pixel

-----------------------------------------------------------------------------
Author : Nicolas Hasbun, nhasbun@gmail.com
File   : vgaHdmi.v
Create : 2017-06-15 15:07:05
Editor : sublime text3, tab size (2)
-----------------------------------------------------------------------------
*/

// **Info Source**
// https://eewiki.net/pages/viewpage.action?pageId=15925278

module vgaHdmi
(
  input clock,     // clk_sys, video timing
  input clk_avr,   // generates oled_clk
  input reset,
  input oled_dc,
  input oled_clk,
  input oled_data,
  input frame_sync,
  input [1:0] greyscale, // off, blend 2 frames, blend 3 frames, checkerboard
  input rate62,          // 250 lines: 62.5 Hz, the Arduboy2 frame rate

  output reg ce_pix,
  output reg hsync, vsync,
  output reg hblank, vblank,
  output     pixelValue,
  output     [2:0] pixelLevel
);

localparam NB = 6;  // up to 4 shown banks; a commit that would overwrite one is dropped

reg  [7:0] mem [1024];
reg  [9:0] waddr, raddr;

reg        invert, segflip, comflip;
reg  [2:0] shiftCount;
reg  [7:0] shiftReg;
wire [7:0] shiftLeft = {shiftReg[6:0], oled_data};
reg  [2:0] pageCount;

// oled_clk is combinational (scl = sck_active & sckint), not a real clock -- sample it
// in clk_avr and edge-detect instead of clocking off it, to avoid a routing-skew runt
// pulse desyncing shiftCount.
reg        oled_clk_d;
wire       oled_clk_rise = ~oled_clk_d & oled_clk;

always @ (posedge clk_avr) begin
  oled_clk_d <= oled_clk;

  if(reset) begin
    waddr         <= 0;
    invert        <= 0;
    segflip       <= 0;
    comflip       <= 0;
    shiftCount    <= 0;
    shiftReg      <= 0;
    pageCount     <= 0;
  end
  else if(oled_clk_rise) begin
    if (oled_dc) begin // data
      if (shiftCount == 3'b111) begin
        mem[{waddr[9:7], waddr[6:0] ^ {7{segflip}}}] <= shiftLeft;
        waddr <= waddr + 1'b1; // Increment address
      end
      else shiftReg <= shiftLeft;
      shiftCount <= shiftCount + 1'b1;
    end
    else begin // commands
      if (shiftCount == 3'b111) begin
        if(shiftLeft[7:1] == 'b1010011) invert <= shiftLeft[0]; // invert (A6/A7)
        if(shiftLeft[7:1] == 'b1010000) segflip <= ~shiftLeft[0]; // segment remap (A0/A1)
        if({shiftLeft[7:4], shiftLeft[2:0]} == 'b1100000) comflip <= ~shiftLeft[3]; // COM scan direction (C0/C8)
        if(shiftLeft[7:3] == 'b10110) waddr <= {shiftLeft[2:0], 7'b000_0000}; // page 0-7 (B0-B7)
        if(shiftLeft == 8'h22) begin // for games using https://github.com/akkera102/08_gamebuino
          waddr <= {pageCount, 7'b000_0000};
          if (pageCount == 3'b101) pageCount <= 3'b000;
          else pageCount <= pageCount + 1'b1;
        end
      end
      else shiftReg <= shiftLeft;
      shiftCount <= shiftCount + 1'b1;
    end
  end
end

// A frame is complete after 1024 data bytes, or 1 ms after the last byte of a shorter burst.
wire       wr_byte = oled_clk_rise & oled_dc & (shiftCount == 3'b111);
reg  [9:0] nbytes;
reg [13:0] idle;
reg        commit = 0;

always @ (posedge clk_avr) begin
  if(reset) begin
    nbytes <= 0;
    idle   <= 0;
  end
  else if(wr_byte) begin
    nbytes <= nbytes + 1'd1;
    idle   <= 0;
    if(&nbytes) commit <= ~commit;
  end
  else if(nbytes) begin
    idle <= idle + 1'd1;
    if(idle == 14'd15999) begin
      nbytes <= 0;
      idle   <= 0;
      commit <= ~commit;
    end
  end
end

wire [7:0] tempByte = mem[raddr];

// Copy each complete frame from mem into the next bank of an NB-bank ring.
wire       latch = frame_sync | |greyscale;
wire       chk   = (greyscale == 2'd3);
wire [2:0] n     = chk ? 3'd4 : (greyscale == 2'd2) ? 3'd3 : 3'd2;
reg  [2:0] commit_s = 0;
reg        pend = 0, copying = 0;
reg [10:0] cidx;
reg  [2:0] ctgt = 0, chead = 0, dhead = 0;
reg        cw_en = 0;
reg  [9:0] cw_addr;

function [2:0] next(input [2:0] b); next = (b == NB - 1) ? 3'd0 : b + 1'd1; endfunction
function [2:0] prev(input [2:0] b); prev = (b == 0) ? NB - 1 : b - 1'd1; endfunction
function shown(input [2:0] b, input [2:0] h, input [2:0] cnt); shown = ((h >= b) ? h - b : h + NB - b) < cnt; endfunction

always @ (posedge clock) begin
  commit_s <= {commit_s[1:0], commit};

  cw_en   <= copying & ~cidx[10];
  cw_addr <= cidx[9:0];

  if(copying) begin
    if(cidx[10]) begin
      copying <= 0;
      chead   <= ctgt;
    end
    else cidx <= cidx + 1'd1;
  end
  else if(pend) begin
    pend <= 0;
    // ring full: drop this frame rather than overwrite one on screen
    if(latch && !shown(next(chead), dhead, n)) begin
      copying <= 1;
      cidx    <= 0;
      ctgt    <= next(chead);
    end
  end

  if(commit_s[2] ^ commit_s[1]) pend <= 1;
end

reg  [9:0] vaddr;
reg  [2:0] vrow;
wire [NB*8-1:0] bank_q;

genvar i;
generate
  for(i = 0; i < NB; i = i + 1) begin : bank
    reg [7:0] ram [1024];
    reg [7:0] q;
    always @ (posedge clock) begin
      if(cw_en && ctgt == i) ram[cw_addr] <= tempByte;
      q <= ram[vaddr];
    end
    assign bank_q[i*8 +: 8] = q;
  end
endgenerate

// head = newest shown bank; lit = pixels set across the n shown banks, lvl = lit/n in quarters
integer    k;
reg  [1:0] lit;
always @(*) begin
  lit = 0;
  for(k = 0; k < NB; k = k + 1)
    if(shown(k[2:0], dhead, n)) lit = lit + bank_q[{k[2:0], vrow}];
end

// checkerboard: grey where the newest frame is a 1-px checker (3x3), or the pixel flips every frame
// (4 frames); newest bank's 3x3 read over the pixel period, other page = row above/below the byte
reg  [5:0] rd;
reg  [7:0] nl, nc, nr, ol, oc;
reg  [2:0] olds;
reg        inner;
wire [7:0] nq  = bank_q[{dhead, 3'b000} +: 8];
wire [9:0] sl  = {ol[0], nl, ol[7]}, sc = {oc[0], nc, oc[7]}, sr = {nq[0], nr, nq[7]};
wire [3:0] vi  = vrow + 4'd1;
wire       px  = sc[vi];
wire       ctr = sc[vi-1] != px && sc[vi+1] != px && sl[vi] != px && sr[vi] != px &&
                 sl[vi-1] == px && sl[vi+1] == px && sr[vi-1] == px && sr[vi+1] == px;
wire       flp = px != olds[0] && px == olds[1] && olds[0] == olds[2];

wire       head = bank_q[{dhead, vrow}];
wire [2:0] lvl  = chk ? ((inner & ctr) | flp ? 3'd2 : {px, 2'b00}) : (n == 3'd2) ? {lit, 1'b0} : (lit == 2'd0) ? 3'd0 : (lit == 2'd1) ? 3'd1 : (lit == 2'd2) ? 3'd3 : 3'd4;

reg ce_pix_pre, ce_pix_int;
always @ (posedge clock) begin
    reg [3:0] div;

    div <= div + 1'd1;

    ce_pix_pre <= !div;
    ce_pix_int <= !div[2:0] & div[3];
    ce_pix     <= !div[2:0]; // twice higher pixel clock to fit OSD on VGA/DV
end

assign pixelValue = pixel & ~vblank_int;
assign pixelLevel = vblank_int ? 3'd0 : level;
reg pixel, vblank_int;
reg [2:0] level;

// Manejo de Pixeles y Sincronizacion
always @ (posedge clock) begin
    reg       invertLatched, comflipLatched;
    reg       old_ce;
    reg       vdiv;
    reg [7:0] pixelH, pixelV;

    old_ce <= ce_pix_pre;

    if (ce_pix_pre) begin
        pixelH <= pixelH + 1'b1;
        if(pixelH == 159) begin
            pixelH <= 0;
            vdiv <= ~vdiv;
            if(vdiv) begin
                pixelV <= pixelV + 1'd1;
                if(pixelV >= (rate62 ? 8'd124 : 8'd130)) begin
                    invertLatched <= invert;
                    comflipLatched <= comflip;
                    pixelV <= 0;
                end
            end
        end
    end

    if (copying) raddr <= cidx[9:0];
    else if (old_ce) raddr <= {pixelV[5:3] ^ {3{comflipLatched}},pixelH[6:0]};
    if (old_ce) {vrow, vaddr} <= {pixelV[2:0] ^ {3{comflipLatched}},pixelV[5:3] ^ {3{comflipLatched}},pixelH[6:0]};

    rd <= {rd[4:0], old_ce};
    if (chk) begin
        if (old_ce) inner <= pixelV[5:0] != 0 && pixelV[5:0] != 63 && pixelH[6:0] != 0 && pixelH[6:0] != 127;
        if (rd[0]) vaddr[6:0] <= pixelH[6:0] - 1'd1;
        if (rd[1]) vaddr[6:0] <= pixelH[6:0] + 1'd1;
        if (rd[2]) vaddr <= {|vrow ? vaddr[9:7] + 1'd1 : vaddr[9:7] - 1'd1, pixelH[6:0] - 1'd1};
        if (rd[3]) vaddr[6:0] <= pixelH[6:0];
        if (rd[4]) vaddr[6:0] <= pixelH[6:0] + 1'd1;
        if (rd[1]) begin
            nc   <= nq;
            olds <= {bank_q[{prev(prev(prev(dhead))), vrow}], bank_q[{prev(prev(dhead)), vrow}], bank_q[{prev(dhead), vrow}]};
        end
        if (rd[2]) nl <= nq;
        if (rd[3]) nr <= nq;
        if (rd[4]) ol <= nq;
        if (rd[5]) oc <= nq;
    end

    if (ce_pix_int) begin
        pixel <= invertLatched ^ (chk ? px : latch ? head : tempByte[pixelV[2:0] ^ {3{comflipLatched}}]);
        level <= invertLatched ? 3'd4 - lvl : lvl;

        if(pixelV == (rate62 ? 8'd121 : 8'd127)) vblank <= 0;
        if(pixelV == 0)   vblank_int <= 0;
        if(pixelV == 64) begin
            vblank_int <= 1;
            dhead      <= chead;
        end
        if(pixelV == 68)  vblank     <= 1;

        vsync  <= (pixelV[7:1] == 47);

        if(pixelH == 0)   hblank <= 0;
        if(pixelH == 128) hblank <= 1;
        if(pixelH == 135) hsync  <= 1;
        if(pixelH == 150) hsync  <= 0;
    end
end

endmodule
