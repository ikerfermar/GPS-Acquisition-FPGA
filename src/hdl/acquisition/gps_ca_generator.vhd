library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- =============================================================
-- gps_ca_generator.vhd
--
-- Generador de codigo C/A GPS L1 segun IS-GPS-200.
-- OPT-2 (Abr 2026): sustituye los LFSR G1/G2 por una ROM BRAM
--   pre-calculada (ca_chips_rom.xci / ca_chips_rom.coe).
--   Direccion: {prn_num[4:0], chip_idx[9:0]} = 15 bits.
--   Ahorra ~150 LUTs a costa de 2 RAMB18.
--
-- La ROM tiene latencia 1 ciclo (output register activo).
-- ca_out se presenta 1 ciclo despues de que chip_idx avanza.
-- epoch_out='1' se emite cuando chip_idx pasa de 1022 a 0
-- (mismo ciclo que el primer chip del nuevo epoch aparece en ca_out
-- un ciclo mas tarde, coherente con el fft_controller).
-- =============================================================

entity gps_ca_generator is
    Port (
        clk       : in  STD_LOGIC;
        clk_en    : in  STD_LOGIC;
        reset     : in  STD_LOGIC;
        prn_num   : in  STD_LOGIC_VECTOR(4 downto 0);
        ca_out    : out STD_LOGIC;
        epoch_out : out STD_LOGIC
    );
end gps_ca_generator;

architecture Behavioral of gps_ca_generator is
    constant CHIP_IDX_LAST : integer := 1022;

    -- Componente ROM inferida desde ca_chips_rom.xci
    COMPONENT ca_chips_rom IS
        PORT (
            clka  : IN  STD_LOGIC;
            addra : IN  STD_LOGIC_VECTOR(14 downto 0);
            douta : OUT STD_LOGIC_VECTOR(0 downto 0)
        );
    END COMPONENT;

    signal chip_idx  : integer range 0 to 1023 := 0;
    signal rom_addr  : std_logic_vector(14 downto 0);
    signal rom_data  : std_logic_vector(0 downto 0);
    signal epoch_r   : std_logic := '0';

begin

    -- Direccion ROM = {prn_num, chip_idx[9:0]}
    rom_addr <= prn_num & std_logic_vector(to_unsigned(chip_idx, 10));

    rom_inst : ca_chips_rom PORT MAP (
        clka  => clk,
        addra => rom_addr,
        douta => rom_data
    );

    -- La ROM tiene 1 ciclo de latencia: ca_out es valido 1 ciclo despues
    -- de que chip_idx se actualiza. El fft_controller captura ca_bit_local
    -- en clk_en_d1, lo que absorbe exactamente esta latencia.
    ca_out    <= rom_data(0);
    epoch_out <= epoch_r;

    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                chip_idx <= 0;
                epoch_r  <= '0';
            else
                epoch_r <= '0';
                if clk_en = '1' then
                    if chip_idx = CHIP_IDX_LAST then
                        chip_idx <= 0;
                        epoch_r  <= '1';
                    else
                        chip_idx <= chip_idx + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

end Behavioral;