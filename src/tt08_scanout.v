`default_nettype none

// Two 20-byte monochrome rows. A row is refilled only after its last use.
module tta_psram_scanout (
    input wire clk, rst_n, pixel_enable, enable, clear_faults,
    input wire [19:0] framebuffer_base,
    input wire color_write,
    input wire color_index,
    input wire [5:0] color_rgb,
    output wire hsync_n, vsync_n,
    output wire [5:0] rgb,
    output wire frame_start,
    output wire memory_valid,
    output wire memory_accept,
    output wire [19:0] memory_addr,
    input wire memory_ready, memory_error,
    input wire [31:0] memory_rdata,
    output reg underrun, fetch_error,
    output wire fetching
);
    wire [9:0] x, y;
    wire active;
    tta_fb_raster raster (
        .clk(clk), .rst_n(rst_n), .pixel_enable(pixel_enable),
        .pixel_x(x), .pixel_y(y), .hsync_n(hsync_n), .vsync_n(vsync_n),
        .active(active), .frame_start(frame_start)
    );

    reg [31:0] row_bank0 [0:4];
    reg [31:0] row_bank1 [0:4];
    reg [5:0] color0, color1;
    reg [1:0] row_valid;
    reg row_good;
    reg [6:0] row_tag0, row_tag1, fill_row;
    reg [2:0] fill_word;
    reg busy, gap, failed;
    reg [19:0] fill_base;

    wire [6:0] display_row = y[8:2];
    wire [6:0] next_row = y == 524 ? 7'd0 : y[8:2] + 1'b1;
    wire [6:0] requested_row = y == 480 ? 7'd0 : y[8:2] + 1'b1;
    wire line_refill = pixel_enable && x == 0 &&
                       (y == 480 || (y < 476 && y[1:0] == 3));
    wire [2:0] display_word = x[9:7];
    wire display_valid = row_valid[display_row[0]] &&
        (display_row[0] ? row_tag1 == display_row : row_tag0 == display_row);
    wire next_valid = row_valid[next_row[0]] &&
        (next_row[0] ? row_tag1 == next_row : row_tag0 == next_row);
    wire [31:0] pixels = display_row[0] ?
        row_bank1[display_word < 5 ? display_word : 0] :
        row_bank0[display_word < 5 ? display_word : 0];
    wire color = pixels[x[6:2]];

    assign rgb = rst_n && enable && active && row_good && display_valid ?
                 (color ? color1 : color0) : 6'b0;
    assign fetching = busy;
    wire slot_free = !active || fill_row[0] != display_row[0] ||
                     (y[1:0] == 3 && display_word > fill_word);
    assign memory_valid = rst_n && busy && !gap;
    assign memory_accept = slot_free;
    assign memory_addr = fill_base + {7'b0, fill_row, 4'b0} +
                         {9'b0, fill_row, 2'b0} + {15'b0, fill_word, 2'b0};

    wire accept = memory_valid && memory_ready;
    always @(posedge clk) begin
        if (accept && !memory_error) begin
            if (fill_row[0]) row_bank1[fill_word] <= memory_rdata;
            else row_bank0[fill_word] <= memory_rdata;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            color0 <= 0; color1 <= 6'b111111;
            row_valid <= 0; row_good <= 0; row_tag0 <= 0; row_tag1 <= 0; fill_row <= 0;
            fill_word <= 0; busy <= 0; gap <= 0; failed <= 0;
            fill_base <= 0; underrun <= 0; fetch_error <= 0;
        end else begin
            if (color_write) begin
                if (color_index) color1 <= color_rgb;
                else color0 <= color_rgb;
            end
            if (clear_faults) begin underrun <= 0; fetch_error <= 0; end
            if (!enable || fill_base != framebuffer_base) begin
                row_valid <= 0;
                row_good <= 0;
            end
            if (pixel_enable && x == 799 &&
                (y == 524 || (y < 479 && y[1:0] == 3))) begin
                row_good <= enable && next_valid;
                if (enable && !next_valid) underrun <= 1;
            end
            if (enable && pixel_enable && x == 0 &&
                (y == 480 || (y < 476 && y[1:0] == 3))) begin
                if (busy) underrun <= 1;
                else if (framebuffer_base[1:0] != 0 ||
                         framebuffer_base > 20'hfda80) fetch_error <= 1;
                else begin
                    fill_base <= framebuffer_base;
                    fill_row <= requested_row;
                    fill_word <= 0;
                    row_valid[requested_row[0]] <= 0;
                    busy <= 1;
                    gap <= 0;
                    failed <= 0;
                end
            end
            if (gap) gap <= 0;
            if (accept) begin
                gap <= 1;
                if (memory_error) begin failed <= 1; fetch_error <= 1; end
                if (fill_word == 4) begin
                    busy <= 0;
                    row_valid[fill_row[0]] <= !failed && !memory_error && enable &&
                                              fill_base == framebuffer_base;
                    if (fill_row[0]) row_tag1 <= fill_row;
                    else row_tag0 <= fill_row;
                end else fill_word <= fill_word + 1'b1;
            end
        end
    end
endmodule

module tta_fb_raster (
    input wire clk, rst_n, pixel_enable,
    output reg [9:0] pixel_x, pixel_y,
    output wire hsync_n, vsync_n, active, frame_start
);
    always @(posedge clk) begin
        if (!rst_n) begin pixel_x <= 0; pixel_y <= 0; end
        else if (pixel_enable) begin
            if (pixel_x == 799) begin
                pixel_x <= 0;
                pixel_y <= pixel_y == 524 ? 10'd0 : pixel_y + 1'b1;
            end else pixel_x <= pixel_x + 1'b1;
        end
    end
    assign active = pixel_x < 640 && pixel_y < 480;
    assign hsync_n = !(pixel_x >= 656 && pixel_x < 752);
    assign vsync_n = !(pixel_y >= 490 && pixel_y < 492);
    assign frame_start = pixel_enable && pixel_x == 0 && pixel_y == 0;
endmodule

`default_nettype wire
