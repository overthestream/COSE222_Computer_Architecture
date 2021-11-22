library verilog;
use verilog.vl_types.all;
entity hazard_detection is
    port(
        ex_memread      : in     vl_logic;
        ex_rd           : in     vl_logic_vector(4 downto 0);
        id_rs1          : in     vl_logic_vector(4 downto 0);
        id_rs2          : in     vl_logic_vector(4 downto 0);
        pc_write        : out    vl_logic;
        if_id_write     : out    vl_logic;
        stall_control   : out    vl_logic
    );
end hazard_detection;
