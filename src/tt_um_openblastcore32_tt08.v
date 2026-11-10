`default_nettype none

module tt_um_openblastcore32_tt08 (
    input wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input wire [7:0] uio_in,
    output wire [7:0] uio_out, uio_oe,
    input wire ena, clk, rst_n
);
    wire cv, ci, cw, cr, ce, cpu_run;
    wire [19:0] ca;
    wire [31:0] cd, cq;
    wire [7:0] cpu_status, cpu_out, cpu_oe;
    tta_cartridge_cpu #(
        .QSPI(3), .SCK_DIV(2), .CLK_DIV(1), .CS_HIGH_CYCLES(4),
        .COMPRESSED(1), .DATA_IO(1), .ENCODING_V2(2), .LOOPBUF(0), .CONT(0)
    ) cpu (
        .clk(clk), .rst_n(rst_n), .run(cpu_run), .ena(ena), .ui_in(8'b0),
        .uo_out(cpu_status), .uio_in(uio_in), .uio_out(cpu_out), .uio_oe(cpu_oe),
        .acc_valid(cv), .acc_io(ci), .acc_write(cw), .acc_addr(ca), .acc_wdata(cd),
        .acc_ready(cr), .acc_error(ce), .acc_rdata(cq)
    );
    wire p_ready, p_result_ready, p_error;
    wire [31:0] p_data;
    wire scan_valid, scan_ready, scan_accept, scan_error, fetching, underrun, fetch_error;
    wire [19:0] scan_addr;
    wire [31:0] scan_data;
    wire [3:0] dq_out, dq_oe;
    wire flash_cs, ram_cs, serial_clock;
    tta_cartridge_memory memory (
        .clk(clk), .rst_n(rst_n), .cpu_cs_n(cpu_out[0]), .cpu_sck(cpu_out[3]),
        .cpu_request_safe(!cv || !ci), .cpu_out({cpu_out[5:4],cpu_out[2:1]}),
        .cpu_oe({cpu_oe[5:4],cpu_oe[2:1]}), .cpu_run(cpu_run),
        .host_valid(cv && !ci), .host_write(cw), .host_addr(ca), .host_wdata(cd),
        .host_ready(p_ready), .host_result_ready(p_result_ready),
        .host_error(p_error), .host_rdata(p_data),
        .scan_valid(scan_valid), .scan_fetching(fetching), .scan_accept(scan_accept),
        .scan_addr(scan_addr),
        .scan_ready(scan_ready), .scan_error(scan_error), .scan_rdata(scan_data),
        .dq_in({uio_in[5:4],uio_in[2:1]}), .dq_out(dq_out), .dq_oe(dq_oe),
        .flash_cs_n(flash_cs), .ram_cs_n(ram_cs), .sck(serial_clock)
    );
    reg video_enable, pixel_enable;
    reg [19:0] framebuffer_base;
    reg [15:0] frame_count;
    (* async_reg="true" *) reg [1:0] gpio_meta, gpio_sync;
    wire frame_start, hs, vs;
    wire [5:0] rgb;
    wire color_select = ci && ca >= 20'h70 && ca <= 20'h71;
    wire base_select = ci && ca == 20'h6a;
    wire control_select = ci && ca == 20'h6b;
    wire status_select = ci && ca == 20'h6c;
    wire frame_select = ci && ca == 20'h10;
    wire gpio_select = ci && ca == 20'h2a;
    wire audio_select = ci && ca >= 20'h30 && ca <= 20'h33;
    wire registers_select = color_select || base_select || control_select ||
        status_select || frame_select || gpio_select;
    wire register_error =
        (color_select && (!cw || |cd[31:6])) ||
        (base_select && cw && (video_enable || |cd[31:20] ||
                               |cd[1:0] || cd[19:0] > 20'hfda80)) ||
        (control_select && cw && |cd[31:2]) ||
        ((status_select || frame_select || gpio_select) && cw);
    wire register_accept = rst_n && cv && registers_select && !register_error;
    wire clear_faults = register_accept && control_select && cw && cd[1];
    tta_psram_scanout video (
        .clk(clk), .rst_n(rst_n), .pixel_enable(pixel_enable),
        .enable(video_enable), .clear_faults(clear_faults),
        .framebuffer_base(framebuffer_base),
        .color_write(register_accept && color_select && cw),
        .color_index(ca[0]), .color_rgb(cd[5:0]),
        .hsync_n(hs), .vsync_n(vs), .rgb(rgb), .frame_start(frame_start),
        .memory_valid(scan_valid), .memory_accept(scan_accept), .memory_addr(scan_addr),
        .memory_ready(scan_ready), .memory_error(scan_error), .memory_rdata(scan_data),
        .underrun(underrun), .fetch_error(fetch_error), .fetching(fetching)
    );
    wire gamepad_select = ci && ca == 20'h20;
    wire gamepad_ready, gamepad_error;
    wire [31:0] gamepad_data;
    phase2_gamepad_io gamepad (
        .clk(clk), .rst_n(rst_n), .pmod_latch(ui_in[4]),
        .pmod_clk(ui_in[5]), .pmod_data(ui_in[6]),
        .acc_valid(cv && gamepad_select), .acc_io(ci), .acc_write(cw),
        .acc_addr(ca), .acc_wdata(cd), .acc_ready(gamepad_ready),
        .acc_error(gamepad_error), .acc_rdata(gamepad_data)
    );
    wire audio_ready, audio_error, audio_out;
    wire [31:0] audio_data;
    tt08_minimal_audio audio_engine (
        .clk(clk), .rst_n(rst_n), .cpu_rst_n(rst_n), .tick(pixel_enable),
        .acc_valid(cv && audio_select), .acc_io(ci), .acc_write(cw),
        .acc_addr(ca), .acc_wdata(cd), .acc_ready(audio_ready),
        .acc_error(audio_error), .acc_rdata(audio_data), .audio(audio_out)
    );
    wire [31:0] register_data = base_select ? {12'b0,framebuffer_base} :
        control_select ? {28'b0,fetching,fetch_error,underrun,video_enable} :
        status_select ? {24'b0,cpu_status} :
        frame_select ? {16'b0,frame_count} :
        gpio_select ? {30'b0,gpio_sync} : 32'b0;
    assign cr = !ci ? p_result_ready : gamepad_select ? gamepad_ready :
                audio_select ? audio_ready : rst_n && cv;
    assign ce = !ci ? p_error : gamepad_select ? gamepad_error :
                audio_select ? audio_error : !registers_select || register_error;
    assign cq = !ci ? p_data : gamepad_select ? gamepad_data :
                audio_select ? audio_data : register_data;
    always @(posedge clk) begin
        if (!rst_n) pixel_enable <= 0;
        else pixel_enable <= !pixel_enable;
    end
    always @(posedge clk) begin
        if (!rst_n) begin gpio_meta <= 0; gpio_sync <= 0; end
        else begin gpio_meta <= ui_in[3:2]; gpio_sync <= gpio_meta; end
    end
    always @(posedge clk) begin
        if (!rst_n) begin video_enable <= 0; framebuffer_base <= 0; frame_count <= 0; end
        else begin
            if (frame_start) frame_count <= frame_count + 1'b1;
            if (register_accept && cw && base_select) framebuffer_base <= cd[19:0];
            if (register_accept && cw && control_select) video_enable <= cd[0];
        end
    end
    assign uo_out = {hs,rgb[0],rgb[2],rgb[4],vs,rgb[1],rgb[3],rgb[5]};
    assign uio_out = {audio_out,ram_cs,dq_out[3:2],serial_clock,dq_out[1:0],flash_cs};
    assign uio_oe = {1'b1,1'b1,dq_oe[3:2],1'b1,dq_oe[1:0],1'b1};
    wire unused_inputs = &{1'b0, ui_in[7], ui_in[1:0]};
endmodule
`default_nettype wire
