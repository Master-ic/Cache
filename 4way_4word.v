// Cache Memory (4way 4word)               //
// i_  means input port                    //
// o_  means output port                   //
// _p_  means data exchange with processor //
// _m_  means data exchange with memory    //
// Replacement policy is LRU (8bit)        //


`default_nettype none

  module cache(clk,
               rst,
               i_p_addr,
               i_p_byte_en,
               i_p_writedata,
               i_p_read,
               i_p_write,
               o_p_readdata,
               o_p_readdata_valid,
               o_p_waitrequest,

               o_m_addr,
               o_m_byte_en,
               o_m_writedata,
               o_m_read,
               o_m_write,
               i_m_readdata,
               i_m_readdata_valid,
               i_m_waitrequest,

               cnt_r,
               cnt_w,
               cnt_hit_r,
               cnt_hit_w,
               cnt_wb_r,
               cnt_wb_w);
// _p_  means data exchange with processor //
// _m_  means data exchange with memory    //
    parameter cache_entry = 14; //缓存中的条目数量
    input wire         clk, rst;
    input wire [24:0]  i_p_addr;                //处理器传来的地址信号，表示处理器想要访问的内存地址
    input wire [3:0]   i_p_byte_en;             //字节使能信号，指示要访问的数据的大小
    input wire [31:0]  i_p_writedata;           //从处理器传来的数据，当处理器想要写入数据到缓存时使用
    input wire         i_p_read, i_p_write;     //分别是读和写使能信号，告诉缓存处理器想要执行的操作是读还是写
    output reg [31:0]  o_p_readdata;            //返回给处理器的数据，当处理器执行读操作并且数据在缓存中时使用
    output reg         o_p_readdata_valid;      //指示 o_p_readdata 中的数据是有效的
    output wire        o_p_waitrequest;         //告诉处理器等待，通常是因为缓存正在处理一个缓存未命中，需要从主内存或下一级缓存获取数据

    output reg [25:0]  o_m_addr;                //缓存发送到主内存或下一级缓存的地址
    output wire [3:0]  o_m_byte_en;             //字节使能信号，告诉主内存或下一级缓存要访问的数据的大小
    output reg [127:0] o_m_writedata;           //缓存写入到主内存或下一级缓存的数据
    output reg         o_m_read, o_m_write;     //读和写使能信号，告诉主内存或下一级缓存缓存想要执行的操作
    input wire [127:0] i_m_readdata;            //从主内存或下一级缓存返回的数据
    input wire         i_m_readdata_valid;      //指示 i_m_readdata 中的数据是有效的
    input wire         i_m_waitrequest;         //主内存或下一级缓存告诉缓存等待，可能是因为它们还在处理前一个请求

    output reg [31:0]  cnt_r;                   //读操作的计数器
    output reg [31:0]  cnt_w;                   //写操作的计数器
    output reg [31:0]  cnt_hit_r;               //读命中的计数器，表示有多少次读操作直接从缓存获取了数据
    output reg [31:0]  cnt_hit_w;               //写命中的计数器
    output reg [31:0]  cnt_wb_r;                //与写回策略相关的计数器，
    output reg [31:0]  cnt_wb_w;                //表示有多少次数据从缓存写回到主内存或下一级缓存

    wire [3:0]    hit;      //是否在该set中命中了一个entry
    wire [3:0]    modify;   //该set中的数据是否被修改过（脏数据）
    wire [3:0]    miss;     //是否未在该set中找到数据
    wire [3:0]    valid;    //该set中的数据是否有效
    wire [127:0]  readdata0, readdata1, readdata2, readdata3; //存储从各个set中读取的数据
    wire [127:0]  writedata;//用于存储要写入缓存的数据
    wire          write0, write1, write2, write3; //控制是否写入对应的set
    wire [3:0]    word_en;  //控制哪些word（32位）要被写入
    wire [3:0] 	  byte_en;  //控制哪些byte（8位）要被写入
    wire [22:0]   addr;     //处理器提供的地址，用于查找缓存
    wire [22:0]   wb_addr0, wb_addr1, wb_addr2, wb_addr3; //write-back地址，表示如果缓存行是脏的，数据应该写回到哪个内存地址
    wire [7:0] 	  r_cm_data; // 从 ram_hot 读取的数据
    wire [1:0] 	  hit_num;   //表示哪个set是命中的

    reg  [2:0] 	  state;
    reg  [127:0]  writedata_buf;        //缓存要写入的数据
    reg  [24:0]   write_addr_buf;       //缓存的地址缓冲，用于存储要写入的地址
    reg  [3:0] 	  byte_en_buf;          //缓冲的byte enable信号
    reg 		  write_buf, read_buf;  //控制信号，用于指示是否要写入或读取缓存
    reg  [3:0]    write_set;            //控制哪个set要被写入
    reg  [3:0]    fetch_write;          //控制从内存中读取数据并写入哪个set
    reg  [7:0] 	  w_cm_data;            //写入 ram_hot 的数据
    reg 		  w_cm;                 //控制是否写入 ram_hot

    localparam IDLE = 0; //空闲
    localparam COMP = 1; //比较
    localparam HIT  = 2; //命中
    localparam FETCH1 = 3; // 数据获取
    localparam FETCH2 = 4; // 数据获取
    localparam FETCH3 = 5; // 数据获取
    localparam WB1 = 6; //写回
    localparam WB2 = 7; //写回


`ifdef SIM
    integer i;
    
    initial begin
        for(i = 0; i <=(2**cache_entry-1); i=i+1) begin
	        ram_hot.mem[i] = 0;
        end
    end
`endif
                                                                        // 13:0                                    13:0
    simple_ram #(.width(8), .widthad(cache_entry)) ram_hot(clk, addr[cache_entry-1:0], w_cm, w_cm_data, addr[cache_entry-1:0], r_cm_data);

    set #(.cache_entry(cache_entry))
    set0(.clk(clk),
         .rst(rst),
         .entry(addr[cache_entry-1:0]),     //要访问的缓存条目（索引）//读地址
         .o_tag(addr[22:cache_entry]),      //地址的标签部分     
         .writedata(writedata),             //要写入缓存的数据
         .byte_en(byte_en),
         .write(write0),
         .word_en(word_en), // 4word r/w change 
         .readdata(readdata0),
         .wb_addr(wb_addr0),
         .hit(hit[0]),
         .modify(modify[0]),
         .miss(miss[0]),
         .valid(valid[0]),
         .read_miss(read_buf));

    set #(.cache_entry(cache_entry))
    set1(.clk(clk),
         .rst(rst),
         .entry(addr[cache_entry-1:0]),
         .o_tag(addr[22:cache_entry]),
         .writedata(writedata),
         .byte_en(byte_en),
         .write(write1),
         .word_en(word_en), // 4word r/w change 
         .readdata(readdata1),
         .wb_addr(wb_addr1),
         .hit(hit[1]),
         .modify(modify[1]),
         .miss(miss[1]),
         .valid(valid[1]),
         .read_miss(read_buf));

    set #(.cache_entry(cache_entry))
    set2(.clk(clk),
         .rst(rst),
         .entry(addr[cache_entry-1:0]),
         .o_tag(addr[22:cache_entry]),
         .writedata(writedata),
         .byte_en(byte_en),
         .write(write2),
         .word_en(word_en), // 4word r/w change 
         .readdata(readdata2),
         .wb_addr(wb_addr2),
         .hit(hit[2]),
         .modify(modify[2]),
         .miss(miss[2]),
         .valid(valid[2]),
         .read_miss(read_buf));

    set #(.cache_entry(cache_entry))
    set3(.clk(clk),
         .rst(rst),
         .entry(addr[cache_entry-1:0]),
         .o_tag(addr[22:cache_entry]),
         .writedata(writedata),
         .byte_en(byte_en),
         .write(write3),
         .word_en(word_en), // 4word r/w change 
         .readdata(readdata3),
         .wb_addr(wb_addr3),
         .hit(hit[3]),
         .modify(modify[3]),
         .miss(miss[3]),
         .valid(valid[3]),
         .read_miss(read_buf));

    assign writedata = (|fetch_write) ?	i_m_readdata : writedata_buf; //128bit
    assign write0 = (fetch_write[0]) ? i_m_readdata_valid : write_set[0];
    assign write1 = (fetch_write[1]) ? i_m_readdata_valid : write_set[1];
    assign write2 = (fetch_write[2]) ? i_m_readdata_valid : write_set[2];
    assign write3 = (fetch_write[3]) ? i_m_readdata_valid : write_set[3];
    assign addr = (o_p_waitrequest) ? write_addr_buf[24:2] : i_p_addr[24:2]; // set module input addr is 23bit 
    assign byte_en = (|fetch_write) ? 4'b1111 : byte_en_buf;
    assign o_p_waitrequest = (state != IDLE);
    assign o_m_byte_en = 4'b1111;//与主存的数据交换总是涉及到全部的4个字节，所以字节使能信号设置为 4'b1111，表示每个字节都是有效的。

    assign hit_num = (hit[0]) ? 0 : (hit[1]) ? 1 : (hit[2]) ? 2 : 3; // 表示在当前访问中命中的set的编号，范围从0到3
    assign word_en = (|fetch_write) ? 4'b1111 : 
                     (write_addr_buf[1:0] == 2'b00) ? 4'b0001 :
                     (write_addr_buf[1:0] == 2'b01) ? 4'b0010 :
                     (write_addr_buf[1:0] == 2'b10) ? 4'b0100 : 4'b1000;

    always @(posedge clk) begin
        if(rst) begin
            o_p_readdata_valid <= 0;
            {o_m_read, o_m_write} <= 0;
            o_m_addr <= 0;
            write_addr_buf <= 0;
            byte_en_buf <= 0;
            writedata_buf <= 0;
            {write_buf, read_buf} <= 0;
            write_set <= 0;
            fetch_write <= 0;
            {cnt_r, cnt_w} <= 0;
            {cnt_hit_r, cnt_hit_w} <= 0;
            {cnt_wb_r, cnt_wb_w} <= 0;
            state <= IDLE;
        end
        else begin
            case (state)
                IDLE: begin
                    write_set <= 0;
                    o_p_readdata_valid <= 0;
                    writedata_buf <= {i_p_writedata, i_p_writedata, i_p_writedata, i_p_writedata};
                    write_addr_buf <= i_p_addr;//处理器想要访问的内存地址
                    byte_en_buf <= i_p_byte_en;
                    write_buf <= i_p_write;
                    read_buf <= i_p_read;
                    if(i_p_read) begin
                        state <= COMP;
                        cnt_r <= cnt_r + 1;
                    end else if(i_p_write) begin
                        state <= COMP;
                        cnt_w <= cnt_w + 1;
                    end
                end
                COMP:  begin
                    if((|hit) && write_buf) begin
                        state <= HIT;
                        write_set <= hit;
                        cnt_hit_w <= cnt_hit_w + 1;
                        //如果 hit_num 与 r_cm_data 的最低2位（即第0 set）匹配，则将 r_cm_data[1:0]（表示最近访问的set）放在最前面，后面跟随其余的位
                        w_cm_data <= (r_cm_data[1:0] == hit_num) ? {r_cm_data[1:0], r_cm_data[7:2]} :
                                     (r_cm_data[3:2] == hit_num) ? {r_cm_data[3:2], r_cm_data[7:4], r_cm_data[1:0]} :
                                     (r_cm_data[5:4] == hit_num) ? {r_cm_data[5:4], r_cm_data[7:6], r_cm_data[3:0]} : r_cm_data;
                    //      最近被访问的set总是放在 w_cm_data 的最前面，而最不常访问的set则放在最后，
                    // 从而实现了一种简化的LRU策略。这是一种常见的方法，用于在组相联缓存中确定哪个set应该被替换。
                        w_cm <= 1;
                    end else if((|hit) && read_buf) begin
                        case(write_addr_buf[1:0])
                            2'b00: o_p_readdata <= (hit[0]) ? readdata0[31:0] : (hit[1]) ? readdata1[31:0] : (hit[2]) ? readdata2[31:0] : readdata3[31:0];
                            2'b01: o_p_readdata <= (hit[0]) ? readdata0[63:32] : (hit[1]) ? readdata1[63:32] : (hit[2]) ? readdata2[63:32] : readdata3[63:32];
                            2'b10: o_p_readdata <= (hit[0]) ? readdata0[95:64] : (hit[1]) ? readdata1[95:64] : (hit[2]) ? readdata2[95:64] : readdata3[95:64];
                            2'b11: o_p_readdata <= (hit[0]) ? readdata0[127:96] : (hit[1]) ? readdata1[127:96] : (hit[2]) ? readdata2[127:96] : readdata3[127:96];
                        endcase
                        o_p_readdata_valid <= 1;
                        w_cm_data <= (r_cm_data[1:0] == hit_num) ? {r_cm_data[1:0], r_cm_data[7:2]} :
                                     (r_cm_data[3:2] == hit_num) ? {r_cm_data[3:2], r_cm_data[7:4], r_cm_data[1:0]} :
                                     (r_cm_data[5:4] == hit_num) ? {r_cm_data[5:4], r_cm_data[7:6], r_cm_data[3:0]} : r_cm_data;
                        w_cm <= 1;
                        cnt_hit_r <= cnt_hit_r + 1;
                        state <= IDLE;
                    end else if(!(&valid) || miss[r_cm_data[1:0]]) begin
                        state <= FETCH1;
                        if(!valid[0]) begin
                            fetch_write <= 4'b0001;
                            w_cm_data <= 8'b11100100;
                            w_cm <= 1;
                        end else if(!valid[1]) begin
                            fetch_write <= 4'b0010;
                            w_cm_data <= (r_cm_data[1:0] == 2'b01) ? {r_cm_data[1:0], r_cm_data[7:2]} :
                                         (r_cm_data[3:2] == 2'b01) ? {r_cm_data[3:2], r_cm_data[7:4], r_cm_data[1:0]} :
                                         (r_cm_data[5:4] == 2'b01) ? {r_cm_data[5:4], r_cm_data[7:6], r_cm_data[3:0]} : r_cm_data;
                            w_cm <= 1;
                        end else if(!valid[2]) begin
                            fetch_write <= 4'b0100;
                            w_cm_data <= (r_cm_data[1:0] == 2'b10) ? {r_cm_data[1:0], r_cm_data[7:2]} :
                                         (r_cm_data[3:2] == 2'b10) ? {r_cm_data[3:2], r_cm_data[7:4], r_cm_data[1:0]} :
                                         (r_cm_data[5:4] == 2'b10) ? {r_cm_data[5:4], r_cm_data[7:6], r_cm_data[3:0]} : r_cm_data;
                            w_cm <= 1;
                        end else if(!valid[3]) begin
                            fetch_write <= 4'b1000;
                            w_cm_data <= (r_cm_data[1:0] == 2'b11) ? {r_cm_data[1:0], r_cm_data[7:2]} :
                                         (r_cm_data[3:2] == 2'b11) ? {r_cm_data[3:2], r_cm_data[7:4], r_cm_data[1:0]} :
                                         (r_cm_data[5:4] == 2'b11) ? {r_cm_data[5:4], r_cm_data[7:6], r_cm_data[3:0]} : r_cm_data;
                            w_cm <= 1;
                        end else if(miss[r_cm_data[1:0]]) begin
                            if(r_cm_data[1:0] == 2'b00) fetch_write <= 4'b0001;
                            else if(r_cm_data[1:0] == 2'b01) fetch_write <= 4'b0010;
                            else if(r_cm_data[1:0] == 2'b10) fetch_write <= 4'b0100;
                            else if(r_cm_data[1:0] == 2'b11) fetch_write <= 4'b1000;
                            w_cm_data <= {r_cm_data[1:0], r_cm_data[7:2]};
                            w_cm <= 1;
                        end
                        o_m_addr <= {write_addr_buf[24:2], 3'b000};
                        o_m_read <= 1;
                    end else begin
                        state <= WB1;
                        if(r_cm_data[1:0] == 2'b00) fetch_write <= 4'b0001;
                        else if(r_cm_data[1:0] == 2'b01) fetch_write <= 4'b0010;
                        else if(r_cm_data[1:0] == 2'b10) fetch_write <= 4'b0100;
                        else if(r_cm_data[1:0] == 2'b11) fetch_write <= 4'b1000;
                        w_cm_data <= {r_cm_data[1:0], r_cm_data[7:2]};
                        w_cm <= 1;
                        if(read_buf) cnt_wb_r <= cnt_wb_r + 1;
                        else if(write_buf) cnt_wb_w <= cnt_wb_w + 1;
                    end
                end
                HIT: begin
                    w_cm <= 0;
                    write_set <= 0;
                    state <= IDLE;
                end //1/13
                FETCH1: begin
                    w_cm <= 0;
                    if(!i_m_waitrequest) begin
                        o_m_read <= 0;
                        state <= FETCH2;
                    end
                end
                FETCH2: begin
                    if(i_m_readdata_valid) begin
                        fetch_write <= 0;            //add 3/9
                        if(write_buf) begin
                            state <= FETCH3;
                            write_set <= fetch_write;
		                end else if(read_buf) begin
                            state <= IDLE;
		                    o_p_readdata_valid <= 1;
		                    case(write_addr_buf[1:0])
		                        2'b00: o_p_readdata <= i_m_readdata[ 31: 0];
		                        2'b01: o_p_readdata <= i_m_readdata[ 63:32];
		                        2'b10: o_p_readdata <= i_m_readdata[ 95:64];
		                        2'b11: o_p_readdata <= i_m_readdata[127:96];
		                    endcase
		                end
                    end
                end
                FETCH3: begin
                    state <= IDLE;
                    write_set <= 0;
                end
                WB1: begin
                    w_cm <= 0;
                    o_m_addr <= (fetch_write[0]) ? {wb_addr0, 3'b000} :
                                (fetch_write[1]) ? {wb_addr1, 3'b000} :
                                (fetch_write[2]) ? {wb_addr2, 3'b000} : {wb_addr3, 3'b000};
                    o_m_writedata <= (fetch_write[0]) ? readdata0 : 
                                     (fetch_write[1]) ? readdata1 : 
                                     (fetch_write[2]) ? readdata2 : readdata3;
                    o_m_write <= 1;
                    state <= WB2;
                end
                WB2: begin
                    if(!i_m_waitrequest) begin
                        o_m_write <= 0;
                        o_m_addr <= {write_addr_buf[24:2], 3'b000};
                        o_m_read <= 1;
                        state <= FETCH1;
                    end
                end
            endcase // case (state)
        end
    end

endmodule // cache

module set(clk,
           rst,
           entry,
           o_tag,
           writedata,
           byte_en,
           write,
           word_en,

           readdata,
           wb_addr,
           hit,
           modify,
           miss,
           valid,
           read_miss);

    parameter cache_entry = 14;//缓存条目的数量

    input wire                    clk, rst;
    input wire [cache_entry-1:0]  entry;//要访问的缓存条目（索引）//读地址
    input wire [22-cache_entry:0] o_tag;//地址的标签部分
    input wire [127:0] 		      writedata;//要写入缓存的数据
    input wire [3:0] 		      byte_en;//字节使能信号，用于指定要写入/读取的字的哪些字节
    input wire       	          write;//写使能
    input wire [3:0]              word_en;//字使能信号，用于指定要写入/读取的缓存行的哪个字
    input wire 			          read_miss;//指示读操作上的缓存未命中的信号

    output wire [127:0] 		  readdata;//从缓存中读取的数据
    output wire [22:0] 		      wb_addr;//写回操作的地址
    output wire 			      hit, modify, miss, valid;

//该模块使用多个简单的RAM（simple_ram）实例来存储缓存数据和标签

    wire [22-cache_entry:0] 	 i_tag;//9位
    wire 			             dirty;
    wire [24-cache_entry:0] 	 write_tag_data;

    assign hit = valid && (o_tag == i_tag);                 //当前标签与缓存中的标签匹配且 缓存行有效 为命中
    assign modify = valid && (o_tag != i_tag) && dirty;     //当前标签不匹配缓存标签 且 缓存行为 脏  为一个已经修改的缓存行
    assign miss = !valid || ((o_tag != i_tag) && !dirty);   //当前缓存行无效 或 当前标签缓存行不是 脏，表示未命中

    assign wb_addr = {i_tag, entry};//写回操作的地址
// 缓存行的数据被分为4个32位的字（word），每个字又被分为4个8位的字节
    //write -> [3:0] write, writedata/readdata 32bit -> 128bit
//第4个字：
    simple_ram #(.width(8), .widthad(cache_entry)) ram11_3(clk, entry, write && word_en[3]  && byte_en[3], writedata[127:120], entry, readdata[127:120]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram11_2(clk, entry, write && word_en[3]  && byte_en[2], writedata[119:112], entry, readdata[119:112]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram11_1(clk, entry, write && word_en[3]  && byte_en[1], writedata[111:104], entry, readdata[111:104]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram11_0(clk, entry, write && word_en[3]  && byte_en[0], writedata[103:96], entry, readdata[103:96]);
//第3个字：
    simple_ram #(.width(8), .widthad(cache_entry)) ram10_3(clk, entry, write && word_en[2]  && byte_en[3], writedata[95:88], entry, readdata[95:88]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram10_2(clk, entry, write && word_en[2]  && byte_en[2], writedata[87:80], entry, readdata[87:80]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram10_1(clk, entry, write && word_en[2]  && byte_en[1], writedata[79:72], entry, readdata[79:72]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram10_0(clk, entry, write && word_en[2]  && byte_en[0], writedata[71:64], entry, readdata[71:64]);
//第2个字：
    simple_ram #(.width(8), .widthad(cache_entry)) ram01_3(clk, entry, write && word_en[1]  && byte_en[3], writedata[63:56], entry, readdata[63:56]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram01_2(clk, entry, write && word_en[1]  && byte_en[2], writedata[55:48], entry, readdata[55:48]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram01_1(clk, entry, write && word_en[1]  && byte_en[1], writedata[47:40], entry, readdata[47:40]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram01_0(clk, entry, write && word_en[1]  && byte_en[0], writedata[39:32], entry, readdata[39:32]);
//第1个字：                                         第**个字节
    simple_ram #(.width(8), .widthad(cache_entry)) ram00_3(clk, entry, write && word_en[0]  && byte_en[3], writedata[31:24], entry, readdata[31:24]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram00_2(clk, entry, write && word_en[0]  && byte_en[2], writedata[23:16], entry, readdata[23:16]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram00_1(clk, entry, write && word_en[0]  && byte_en[1], writedata[15: 8], entry, readdata[15:8]);
    simple_ram #(.width(8), .widthad(cache_entry)) ram00_0(clk, entry, write && word_en[0]  && byte_en[0], writedata[ 7: 0], entry, readdata[ 7:0]);


    assign write_tag_data = (read_miss) ? {1'b0, 1'b1, o_tag} : 
                            (modify || miss ) ? {1'b1, 1'b1, o_tag} :
                                                {1'b1, 1'b1, i_tag};
                    ///////////11位                                                                                  1      1      9
    simple_ram #(.width(25-cache_entry), .widthad(cache_entry)) ram_tag(clk, entry, write, write_tag_data, entry, {dirty, valid, i_tag});

`ifdef SIM
    integer i;

    initial begin
        for(i = 0; i <=(2**cache_entry-1); i=i+1) begin
	        ram_tag.mem[i] = 0;
        end
    end
`endif

endmodule
