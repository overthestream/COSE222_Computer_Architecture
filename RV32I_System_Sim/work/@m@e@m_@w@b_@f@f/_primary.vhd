library verilog;
use verilog.vl_types.all;
entity MEM_WB_FF is
    port(
        read_data       : in     vl_logic_vector(31 downto 0);
        address         : in     vl_logic_vector(31 downto 0);
        wb_pc           : in     vl_logic_vector(31 downto 0);
        rd              : in     vl_logic_vector(4 downto 0);
        control         : in     vl_logic_vector(2 downto 0);
        clk             : in     vl_logic;
        control_out     : out    vl_logic_vector(2 downto 0);
        read_data_out   : out    vl_logic_vector(31 downto 0);
        address_out     : out    vl_logic_vector(31 downto 0);
        wb_pc_out       : out    vl_logic_vector(31 downto 0);
        rd_out          : out    vl_logic_vector(4 downto 0)
    );
end MEM_WB_FF;
