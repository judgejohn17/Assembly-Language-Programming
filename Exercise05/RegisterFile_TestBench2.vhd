-------------------------------------------------
-- File:	RegisterFile_TestBench.vhd
-- Entity:	RegisterFile_TestBeench
-- Architecture:	Behavioral
-- Author: John Judge
-- Created: 2/28/16
-- Modified: 2/28/16
-- VHDL'93
-- Description: The following is the Test Bench
-- For the Register File created in this project.
-- This tests the functionality of the design using
-- a few test cases.
-------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;
USE ieee.std_logic_unsigned.all;
 
ENTITY RegisterFile_TestBench IS
	--Test Bench entity has no ports 
END RegisterFile_TestBench;
 
ARCHITECTURE behavior OF RegisterFile_TestBench IS 
    -- Component Declaration for the Unit Under Test (UUT)
    COMPONENT RegisterFile
    PORT(
         RD1 : IN  std_logic_vector(2 downto 0);
         RD2 : IN  std_logic_vector(2 downto 0);
         WR : IN  std_logic_vector(2 downto 0);
         OUT1 : OUT  std_logic_vector(15 downto 0);
         OUT2 : OUT  std_logic_vector(15 downto 0);
         INPUT : IN  std_logic_vector(15 downto 0);
         WE : IN  std_logic;
         RESET : IN  std_logic;
         CLOCK : IN  std_logic
        );
    END COMPONENT;
	 
   --Local Signals and Constansts
	
   --Inputs
   signal RD1 : std_logic_vector(2 downto 0) := (others => '0');
   signal RD2 : std_logic_vector(2 downto 0) := (others => '0');
   signal WR : std_logic_vector(2 downto 0) := (others => '0');
   signal INPUT : std_logic_vector(15 downto 0) := (others => '0');
   signal WE : std_logic := '0';
   signal RESET : std_logic := '0';
   signal CLOCK : std_logic := '0';

 	--Outputs
   signal OUT1 : std_logic_vector(15 downto 0);
   signal OUT2 : std_logic_vector(15 downto 0);

   -- Clock period definitions
   constant CLOCK_period : time := 10 ns;
	CONSTANT RESET_WIDTH : TIME := 40 NS;
	CONSTANT WAIT10 : TIME := 10 NS;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: RegisterFile PORT MAP (
          RD1 => RD1,
          RD2 => RD2,
          WR => WR,
          OUT1 => OUT1,
          OUT2 => OUT2,
          INPUT => INPUT,
          WE => WE,
          RESET => RESET,
          CLOCK => CLOCK
        );

   -- Clock process definitions
   CLOCK_process :process
   begin
		CLOCK <= '0';
		wait for CLOCK_period/2;
		CLOCK <= '1';
		wait for CLOCK_period/2;
   end process;
 

   -- Stimulus process
   Test_Bench_Process : process
   begin	
		----------------------------------------
      -- hold reset state for 100 ns.
		Reset <= '1'; --reset is active high
      wait for 100 NS;
		----------------------------------------
		----------------------------------------
		--WRITE TO EACH REGISTER
		RESET <= '1'; --Need to write, reset must be low 		
		WAIT FOR WAIT10;
		WE <= '1';--Initial value, Write Enable is 1 because I want to write 
		WR <= "000";--Initial value of 0, First register is 0
		INPUT <= "0000000000000000";--Initial value of 0, will increment by 1 for testing
		RD1 <= "000"; --Intital value of 0
		RD2 <= "000"; --Initial Value of 0
		FOR I IN 0 TO 6 LOOP--Loop for WR
			WR <= WR+1; --Increment Wrtie Select 
			INPUT <= INPUT+1; --Increment input
			
			
			WAIT FOR WAIT10; --Wait
		END LOOP;
		WE <= '0'; --Stop Writing
		WAIT FOR WAIT10; -- Wait a bit 
		----------------------------------------
		----------------------------------------
		--READ EACH REGISTER
		RD1 <= "000"; --Intital value of 0
		RD2 <= "000"; --Initial Value of 0
		FOR K IN 0 TO 7 LOOP -- Loop for reading
			RD1 <= RD1 +1; --Increment what register is being read
			WAIT FOR WAIT10; -- Wait a bit
			RD2 <= RD2 +1; --Increment what register is being read
		END LOOP;
		RD1 <= "000"; --Intital value of 0
		RD2 <= "000"; --Initial Value of 0
		WAIT FOR WAIT10; --Wait a bit
		----------------------------------------
      -- HOLD RESET
		Reset <= '0'; --reset is active high
     
		----------------------------------------
		----------------------------------------
		WE <= '0';--Initial value, Write Enable is 1 because I want to write 
		WR <= "000";--Initial value of 0, First register is 0
		INPUT <= "0000000000000000";--Initial value of 0, will increment by 1 for testing
		RD1 <= "000"; --Intital value of 0
		RD2 <= "000"; --Initial Value of 0
		FOR I IN 0 TO 6 LOOP--Loop for WR
			WR <= WR+1; --Increment Wrtie Select 
			INPUT <= INPUT+1; --Increment input
			
			
			WAIT FOR WAIT10; --Wait
		END LOOP;
		WE <= '0'; --Stop Writing
		WAIT FOR WAIT10; -- Wait a bit 		
		----------------------------------------
      -- HOLD RESET
		Reset <= '0'; --reset is active high
      wait;
		----------------------------------------
		----------------------------------------
	
			

      -- insert stimulus here 

      wait;
   end process;

END;
