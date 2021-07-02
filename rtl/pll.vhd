----------------------------------------------------------------------------------
-- MiSTer2MEGA65 Framework  
--
-- Main clock, pixel clock and QNICE-clock generator using the Xilinx specific MMCME2_ADV:
--
--   @TODO YOURCORE expects 40 MHz
--   QNICE expects 50 MHz
--   HDMI 720p 60 Hz expects 74.25 MHz (VGA) and 371.25 MHz (HDMI)
--
-- MiSTer2MEGA65 done by sy2002 and MJoergen in 2021 and licensed under GPL v3
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library unisim;
use unisim.vcomponents.all;

entity pll is
   port (
		refclk            : in  std_logic;
		rst               : in  std_logic;
		outclk_0          : out std_logic;
		outclk_1          : out std_logic;
		outclk_2          : out std_logic;
		locked            : out std_logic;
		reconfig_to_pll   : in  std_logic_vector(63 downto 0);
		reconfig_from_pll : out std_logic_vector(63 downto 0)
   );
end pll;

architecture rtl of pll is

   signal clkfb         : std_logic;
   signal clkfb_mmcm    : std_logic;
   signal outclk_0_mmcm : std_logic;
   signal outclk_1_mmcm : std_logic;
   signal outclk_2_mmcm : std_logic;

begin

   i_mmcm : MMCME2_ADV
      generic map (
         BANDWIDTH            => "OPTIMIZED",
         CLKOUT4_CASCADE      => FALSE,
         COMPENSATION         => "ZHOLD",
         STARTUP_WAIT         => FALSE,
         CLKIN1_PERIOD        => 20.0,       -- INPUT @ 50 MHz
         REF_JITTER1          => 0.010,
         DIVCLK_DIVIDE        => 3,
         CLKFBOUT_MULT_F      => 56.750,     -- 945.833 MHz
         CLKFBOUT_PHASE       => 0.000,
         CLKFBOUT_USE_FINE_PS => FALSE,
         CLKOUT0_DIVIDE_F     => 20.000,     -- 47.292 MHz
         CLKOUT0_PHASE        => 0.000,
         CLKOUT0_DUTY_CYCLE   => 0.500,
         CLKOUT0_USE_FINE_PS  => FALSE,
         CLKOUT1_DIVIDE       => 15,         -- 63.056 MHz
         CLKOUT1_PHASE        => 0.000,
         CLKOUT1_DUTY_CYCLE   => 0.500,
         CLKOUT1_USE_FINE_PS  => FALSE,
         CLKOUT2_DIVIDE       => 30,         -- 31.528 MHz
         CLKOUT2_PHASE        => 0.000,
         CLKOUT2_DUTY_CYCLE   => 0.500,
         CLKOUT2_USE_FINE_PS  => FALSE
      )
      port map (
         -- Output clocks
         CLKFBOUT            => clkfb_mmcm,
         CLKOUT0             => outclk_0_mmcm,
         CLKOUT1             => outclk_1_mmcm,
         CLKOUT2             => outclk_2_mmcm,
         -- Input clock control
         CLKFBIN             => clkfb,
         CLKIN1              => refclk,
         CLKIN2              => '0',
         -- Tied to always select the primary input clock
         CLKINSEL            => '1',
         -- Ports for dynamic reconfiguration
         DADDR               => (others => '0'),
         DCLK                => '0',
         DEN                 => '0',
         DI                  => (others => '0'),
         DO                  => open,
         DRDY                => open,
         DWE                 => '0',
         -- Ports for dynamic phase shift
         PSCLK               => '0',
         PSEN                => '0',
         PSINCDEC            => '0',
         PSDONE              => open,
         -- Other control and status signals
         LOCKED              => open,
         CLKINSTOPPED        => open,
         CLKFBSTOPPED        => open,
         PWRDWN              => '0',
         RST                 => rst
      );


   -------------------------------------
   -- Output buffering
   -------------------------------------

   clkfb_bufg : BUFG
      port map (
         I => clkfb_mmcm,
         O => clkfb
      );

   outclk_0_bufg : BUFG
      port map (
         I => outclk_0_mmcm,
         O => outclk_0
      );

   outclk_1_bufg : BUFG
      port map (
         I => outclk_1_mmcm,
         O => outclk_1
      );

   outclk_2_bufg : BUFG
      port map (
         I => outclk_2_mmcm,
         O => outclk_2
      );

end architecture rtl;

