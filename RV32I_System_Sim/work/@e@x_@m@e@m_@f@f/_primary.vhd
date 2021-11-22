library verilog;
use verilog.vl_types.all;
entity EX_MEM_FF is
    port(
        pc              : in     vl_logic_vector(31 downto 0);
        branch          : in     vl_logic_vector(31 downto 0);
        jal             : in     vl_logic_vector(31 downto 0);
        jalr            : in     vl_logic_vector(31 downto 0);
        wr_data_in      : in     vl_logic_vector(31 downto 0);
        result          : in     vl_logic_vector(31 downto 0);
        rd              : in     vl_logic_vector(4 downto 0);
        control         : in     vl_logic_vector(5 downto 0);
        zflag           : in     vl_logic;
        clk             : in     vl_logic;
        en              : in     vl_logic;
        zflag_out       : out    vl_logic;
        rd_out          : out    vl_logic_vector(4 downto 0);
        control_out     : out    vl_logic_vector(5 downto 0);
        pc_out          : out    vl_logic_vector(31 downto 0);
        branch_out      : out    vl_logic_vector(31 downto 0);
        jal_out         : out    vl_logic_vector(31 downto 0);
        jalr_out        : out    vl_logic_vector(31 downto 0);
        wr_data_out     : out    vl_logic_vector(31 downto 0);
        result_out      : out    vl_logic_vector(31 downto 0)
    );
end EX_MEM_FF;
