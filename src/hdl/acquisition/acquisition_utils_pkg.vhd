library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package acquisition_utils_pkg is
    function cfar_scale(noise : unsigned(23 downto 0); k : integer) return unsigned;
    function map_peak_to_phase(pos : std_logic_vector(9 downto 0); bias : integer) return std_logic_vector;
    function abs_signed_bin_diff(a : std_logic_vector(7 downto 0); b : std_logic_vector(7 downto 0)) return integer;
    function phase_circular_distance(a : std_logic_vector(9 downto 0); b : std_logic_vector(9 downto 0)) return integer;
    function next_enabled_prn(mask : std_logic_vector(31 downto 0); start_idx : integer; lim : integer) return integer;
end package;

package body acquisition_utils_pkg is
    constant CHIP_COUNT : integer := 1024;
    constant CHIP_LAST  : integer := CHIP_COUNT - 1;
    constant CHIP_HALF  : integer := CHIP_COUNT / 2;

    function cfar_scale(noise : unsigned(23 downto 0); k : integer) return unsigned is
        variable acc : unsigned(31 downto 0);
    begin
        if k = 0 then
            return not to_unsigned(0, 24);
        elsif k = 1 then
            return noise;
        elsif k = 2 then
            return shift_left(noise, 1);
        elsif k = 3 then
            return shift_left(noise, 1) + noise;
        elsif k = 4 then
            return shift_left(noise, 2);
        elsif k = 5 then
            return shift_left(noise, 2) + noise;
        elsif k = 6 then
            return shift_left(noise, 2) + shift_left(noise, 1);
        elsif k = 7 then
            return shift_left(noise, 3) - noise;
        elsif k = 8 then
            return shift_left(noise, 3);
        elsif k = 9 then
            return shift_left(noise, 3) + noise;
        elsif k = 10 then
            return shift_left(noise, 3) + shift_left(noise, 1);
        elsif k = 12 then
            return shift_left(noise, 3) + shift_left(noise, 2);
        elsif k = 16 then
            return shift_left(noise, 4);
        elsif k = 18 then
            return shift_left(noise, 4) + shift_left(noise, 1);
        else
            acc := (others => '0');
            for i in 1 to 31 loop
                if i <= k then
                    acc := acc + resize(noise, 32);
                end if;
            end loop;
            return acc(23 downto 0);
        end if;
    end function;

    function map_peak_to_phase(pos : std_logic_vector(9 downto 0); bias : integer) return std_logic_vector is
        variable p        : integer;
        variable phase_v  : integer;
        variable phase_n  : integer;
    begin
        p := to_integer(unsigned(pos));
        phase_v := (CHIP_LAST - p) + bias;
        phase_n := phase_v mod CHIP_COUNT;
        if phase_n < 0 then
            phase_n := phase_n + CHIP_COUNT;
        end if;
        return std_logic_vector(to_unsigned(phase_n, 10));
    end function;

    function abs_signed_bin_diff(a : std_logic_vector(7 downto 0); b : std_logic_vector(7 downto 0)) return integer is
        variable ai     : integer;
        variable bi     : integer;
        variable diff_i : integer;
    begin
        ai := to_integer(signed(a));
        bi := to_integer(signed(b));
        diff_i := ai - bi;
        if diff_i < 0 then
            diff_i := -diff_i;
        end if;
        return diff_i;
    end function;

    function phase_circular_distance(a : std_logic_vector(9 downto 0); b : std_logic_vector(9 downto 0)) return integer is
        variable ai     : integer;
        variable bi     : integer;
        variable diff_i : integer;
    begin
        ai := to_integer(unsigned(a));
        bi := to_integer(unsigned(b));
        diff_i := ai - bi;
        if diff_i < 0 then
            diff_i := -diff_i;
        end if;
        if diff_i > CHIP_HALF then
            diff_i := CHIP_COUNT - diff_i;
        end if;
        return diff_i;
    end function;

    function next_enabled_prn(mask : std_logic_vector(31 downto 0); start_idx : integer; lim : integer) return integer is
    begin
        for i in 0 to 31 loop
            if i >= start_idx and i < lim then
                if mask(i) = '1' then
                    return i;
                end if;
            end if;
        end loop;
        return 32;
    end function;
end package body;