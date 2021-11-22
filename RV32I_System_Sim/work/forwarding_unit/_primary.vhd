library verilog;
use verilog.vl_types.all;
entity forwarding_unit is
    port(
        ex_rs1          : in     vl_logic_vector(4 downto 0);
        ex_rs2          : in     vl_logic_vector(4 downto 0);
        mem_rd          : in     vl_logic_vector(4 downto 0);
        wb_rd           : in     vl_logic_vector(4 downto 0);
        regwrite_mem    : in     vl_logic;
        regwrite_wb     : in     vl_logic;
        use_imm         : in     vl_logic;
        fw_rs1_mem      : out    vl_logic;
        fw_rs2_mem      : out    vl_logic;
        fw_rs1_wb       : out    vl_logic;
        fw_rs2_wb       : out    vl_logic
    );
end forwarding_unit;
