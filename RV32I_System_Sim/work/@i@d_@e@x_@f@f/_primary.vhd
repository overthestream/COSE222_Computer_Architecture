library verilog;
use verilog.vl_types.all;
entity ID_EX_FF is
    port(
        pc              : in     vl_logic_vector(31 downto 0);
        imm_u           : in     vl_logic_vector(31 downto 0);
        imm_s           : in     vl_logic_vector(31 downto 0);
        imm_i           : in     vl_logic_vector(31 downto 0);
        rs1             : in     vl_logic_vector(31 downto 0);
        rs2             : in     vl_logic_vector(31 downto 0);
        imm_jal         : in     vl_logic_vector(31 downto 0);
        imm_br          : in     vl_logic_vector(31 downto 0);
        rd              : in     vl_logic_vector(4 downto 0);
        rsreg1          : in     vl_logic_vector(4 downto 0);
        rsreg2          : in     vl_logic_vector(4 downto 0);
        control         : in     vl_logic_vector(13 downto 0);
        clk             : in     vl_logic;
        en              : in     vl_logic;
        id_memread      : in     vl_logic;
        ex_memread      : out    vl_logic;
        control_out     : out    vl_logic_vector(13 downto 0);
        pc_out          : out    vl_logic_vector(31 downto 0);
        imm_u_out       : out    vl_logic_vector(31 downto 0);
        imm_s_out       : out    vl_logic_vector(31 downto 0);
        imm_i_out       : out    vl_logic_vector(31 downto 0);
        rs1_out         : out    vl_logic_vector(31 downto 0);
        rs2_out         : out    vl_logic_vector(31 downto 0);
        imm_jal_out     : out    vl_logic_vector(31 downto 0);
        imm_br_out      : out    vl_logic_vector(31 downto 0);
        rd_out          : out    vl_logic_vector(4 downto 0);
        rsreg1_out      : out    vl_logic_vector(4 downto 0);
        rsreg2_out      : out    vl_logic_vector(4 downto 0)
    );
end ID_EX_FF;
