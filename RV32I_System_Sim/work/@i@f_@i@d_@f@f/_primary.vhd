library verilog;
use verilog.vl_types.all;
entity IF_ID_FF is
    port(
        pc              : in     vl_logic_vector(31 downto 0);
        inst            : in     vl_logic_vector(31 downto 0);
        clk             : in     vl_logic;
        en              : in     vl_logic;
        inst_out        : out    vl_logic_vector(31 downto 0);
        pc_out          : out    vl_logic_vector(31 downto 0)
    );
end IF_ID_FF;
