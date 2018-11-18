
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
entity CPU is
    port(
        clk,rst: in std_logic;
        R4_out: out std_logic_vector(7 downto 0));
    
end CPU;

architecture Behavioral of CPU is
-- ________________ PROGRAM COUNTER _____________________
component PC is
    port(
        clk,rst,en: in std_logic;
        PC_in : in std_logic_vector(31 downto 0);
        PC_out : out std_logic_vector(31 downto 0));
end component;
-- ________________ INSTRUCTION MEMORY _____________________
component InstMem is
    port(
        clk: in std_logic;
        ad: in std_logic_vector(31 downto 0);
        int: out std_logic_vector(31 downto 0));
end component;
-- ________________ IMMEDIATE GENERATION _____________________
component ImmGen is
    Port( 
        inst: in std_logic_vector(31 downto 0);
        imm: out std_logic_vector(31 downto 0));
end component;
-- ________________ REGISTER FILE _____________________
component REG_N is
    port(
        clk,rst,rwr: in std_logic;
        wr_dt: in std_logic_vector( 31 downto 0);
        wr: in std_logic_vector( 4 downto 0);
        rd1: in std_logic_vector( 4 downto 0);
        rd2: in std_logic_vector( 4 downto 0);
        do4_out: out std_logic_vector( 31 downto 0);
        dt1: out std_logic_vector( 31 downto 0);
        dt2: out std_logic_vector( 31 downto 0));
end component;
-- ________________ ALU CONTROL _____________________
component ALU_Control is
    port(
        inst:in std_logic_vector(31 downto 0);
        control: out std_logic_vector(3 downto 0));
end component;
-- ________________ ALU  _____________________
component ALU is
    port(
    A,B,less:in std_logic_vector(31 downto 0);
    control: in std_logic_vector(3 downto 0);
    result: out std_logic_vector(31 downto 0);
    cout,zero: out std_logic);
end component;
-- ________________ DATA MEMORY  _____________________
component DMem is
    port(
        clk,dmw,dmr: in std_logic;
        ad: in std_logic_vector( 31 downto 0);
        wr_dt: in std_logic_vector( 31 downto 0);
        do: out std_logic_vector( 31 downto 0));
end component;
-- ________________ CONTROL_UNIT _______________________
component Control_Unit is
    port(
        opcode: in std_logic_vector(6 downto 0);
        dmw,dmr,memreg,rwr,ALUsrc,branch:out std_logic);
end component;
-- _________________ SIGNALS __________________________
signal do4_out: std_logic_vector( 31 downto 0);
signal PC_in : std_logic_vector(31 downto 0);
signal PC_in_regular: std_logic_vector(31 downto 0);
signal PC_in_branch: std_logic_vector(31 downto 0);
signal PC_out: std_logic_vector(31 downto 0);
signal inst: std_logic_vector(31 downto 0);
signal imm: std_logic_vector(31 downto 0);
signal control: std_logic_vector(3 downto 0);

--- _________________ALU inputs _________________________
signal dt1: std_logic_vector( 31 downto 0);
signal dt2: std_logic_vector( 31 downto 0);
signal A: std_logic_vector( 31 downto 0);
signal B: std_logic_vector( 31 downto 0);
signal B_forward: std_logic_vector( 31 downto 0);
signal ALU_result: std_logic_vector( 31 downto 0);
signal wr: std_logic_vector( 4 downto 0);
signal wr_dt: std_logic_vector( 31 downto 0);
signal rd1: std_logic_vector( 4 downto 0);
signal rd2: std_logic_vector( 4 downto 0);

-- _________________ DATA MEM __________________________
signal do: std_logic_vector( 31 downto 0);

--_____________Control Signals ______________________
signal ALUsrc: std_logic;
signal cout: std_logic;
signal zero: std_logic;
signal rwr: std_logic;
signal memreg: std_logic;
-- data memory
signal dmw: std_logic;
signal dmr: std_logic;

-- branch 
signal PCsrc: std_logic;
signal branch: std_logic;
-- stalling signal 
signal stall: std_logic;
signal en: std_logic;
--_____________________________________________________


-- %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
----result instruction fetch stage  IF_ID reg_out
signal PC_out_if_id: std_logic_vector(31 downto 0);
signal inst_if_id: std_logic_vector(31 downto 0);

----result instruction decode stage ID_EX reg_out
signal dt1_id_ex: std_logic_vector( 31 downto 0);
signal dt2_id_ex: std_logic_vector( 31 downto 0);
signal imm_id_ex: std_logic_vector(31 downto 0);
signal control_id_ex: std_logic_vector(3 downto 0);
signal PC_out_id_ex: std_logic_vector(31 downto 0);
signal rd1_id_ex: std_logic_vector( 4 downto 0);
signal rd2_id_ex: std_logic_vector( 4 downto 0);
signal wr_id: std_logic_vector( 4 downto 0); --- id stage
signal wr_id_ex: std_logic_vector( 4 downto 0);
--___________________ Control signals _________________--
signal branch_id_ex: std_logic;
signal dmw_id_ex: std_logic;
signal dmr_id_ex: std_logic;
signal ALUsrc_id_ex: std_logic;
signal rwr_id_ex: std_logic;
signal memreg_id_ex: std_logic;

----result Execuation stage EX_MEM reg_out
signal PC_in_branch_ex_mem: std_logic_vector(31 downto 0);
signal ALU_result_ex_mem: std_logic_vector( 31 downto 0);
signal dt2_ex_mem: std_logic_vector( 31 downto 0);
signal wr_ex_mem: std_logic_vector( 4 downto 0);
--___________________ Control signals _________________--
signal zero_ex_mem: std_logic;
signal branch_ex_mem: std_logic;
signal dmw_ex_mem: std_logic;
signal dmr_ex_mem: std_logic;
signal rwr_ex_mem: std_logic;
signal memreg_ex_mem: std_logic;

----result Execuation stage MEM_WB reg_out
signal ALU_result_mem_wb: std_logic_vector( 31 downto 0);
signal wr_mem_wb: std_logic_vector( 4 downto 0);
signal do_mem_wb: std_logic_vector( 31 downto 0);
--___________________ Control signals _________________--
signal rwr_mem_wb: std_logic;
signal memreg_mem_wb: std_logic;

-- result of WB and Alu stage
signal wr_dt_wb_alu: std_logic_vector( 31 downto 0);
signal wr_wb_alu: std_logic_vector( 4 downto 0);
--___________________ Control signals _________________--
signal rwr_wb_alu: std_logic;

--_________ HAZARD signals_________
signal forward_B: std_logic_vector( 1 downto 0);
signal forward_A: std_logic_vector( 1 downto 0);

--_______________________________________________________________________________________________________________
begin

PC1: PC port map(clk=>clk,rst=>rst,en=>en,PC_in=>PC_in,PC_out=>PC_out);

InstMem1:InstMem port map(clk=>clk,ad=>PC_out,int=>inst); -- Indtrction Fetch stage
ImmGen1:ImmGen port map(inst=>inst_if_id,imm=>imm); -- Instruction Decode stage

CU: Control_Unit port map(opcode=>inst_if_id(6 downto 0),dmw=>dmw,dmr=>dmr,memreg=>memreg,rwr=>rwr,ALUsrc=>ALUsrc,branch=>branch);

REG1: REG_N port map(clk=>clk,rst=>rst,rwr=>rwr_mem_wb,wr_dt=>wr_dt,wr=>wr_mem_wb,rd1=>rd1,rd2=>rd2,dt1=>dt1,dt2=>dt2,do4_out=>do4_out);

ALU_Control1: ALU_Control port map(inst=>inst_if_id,control=>control); -- Instruction Decode stage
ALU1: ALU port map(A=>A,B=>B,less=>B,control=>control_id_ex,result=>ALU_result,cout=>cout,zero=>zero); -- Execution state


DMem1: DMem port map(clk=>clk,dmw=>dmw_ex_mem,dmr=>dmr_ex_mem,ad=>ALU_result_ex_mem,wr_dt=>dt2_ex_mem,do=>do); -- Memory write stage

-- ____________________ CONCURENT LOGIC _______________________
--_____________ input to PC _________________________ updated
PCsrc <= (branch_ex_mem and zero_ex_mem);

PC_in_branch <= (imm_id_ex(30 downto 0) & '0') + PC_out_id_ex;
PC_in_regular <= PC_out + 4;

PC_in <= PC_in_regular when PCsrc = '0' else
            PC_in_branch_ex_mem;
        
--___________WRITE  BACK _____Input to Registers __________
rd1 <= inst_if_id(19 downto 15); --updated
rd2 <= inst_if_id(24 downto 20); --updated
wr_id <= inst_if_id(11 downto 7);  --updated

--%%%%%%%%%%%%% IF_ID STAGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
IF_ID: process(clk,rst)
begin
    if(rst = '1') then
        inst_if_id<=(others=>'0');
        PC_out_if_id <=(others=>'0');
    elsif(rising_edge(clk) and (en='1')) then 
        if(PCsrc = '0') then
            inst_if_id<=inst;
            PC_out_if_id <= PC_out;
        else
            inst_if_id<=(others=>'0');
            PC_out_if_id <= (others=>'0');
        end if;
    end if;
end process;
--%%%%%%%%%%%%% ID_EX STAGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ID_EX: process(clk,rst)
begin
    if(rst = '1') then
        -- data_line
        dt1_id_ex <= (others=>'0');
        dt2_id_ex <= (others=>'0');
        imm_id_ex <= (others=>'0');
        control_id_ex <= (others=>'0');
        PC_out_id_ex <= (others=>'0');
        wr_id_ex <= (others=>'0');
        rd1_id_ex <= (others=>'0');
        rd2_id_ex <= (others=>'0');
        --control signals
        --WB
        rwr_id_ex <= '0';
        memreg_id_ex <= '0';
        -- EX
        ALUsrc_id_ex <= '0';
        -- mem
        branch_id_ex <= '0';
        dmr_id_ex <= '0';
        dmw_id_ex <= '0';      
    elsif(rising_edge(clk)) then 
        if((en = '1') and (PCsrc = '0')) then
            -- data_line
            dt1_id_ex <= dt1;
            dt2_id_ex <= dt2;
            imm_id_ex <= imm;
            control_id_ex <= control;
            PC_out_id_ex <= PC_out_if_id;
            wr_id_ex <= wr_id;
            rd1_id_ex <= rd1;
            rd2_id_ex <= rd2;
            --control signals
            --WB
            rwr_id_ex <= rwr;
            memreg_id_ex<= memreg;
            -- EX
            ALUsrc_id_ex <= ALUsrc;
            -- MEM
            branch_id_ex <= branch;
            dmr_id_ex <= dmr;
            dmw_id_ex <= dmw;
        else
            -- data_line
             dt1_id_ex <= (others=>'0');
             dt2_id_ex <= (others=>'0');
             imm_id_ex <= (others=>'0');
             control_id_ex <= (others=>'0');
             PC_out_id_ex <= (others=>'0');
             wr_id_ex <= (others=>'0');
             
             --control signals
             --WB
             rwr_id_ex <= '0';
             memreg_id_ex <= '0';
             -- EX
             ALUsrc_id_ex <= '0';
             -- mem
             branch_id_ex <= '0';
             dmr_id_ex <= '0';
             dmw_id_ex <= '0'; 
             
         end if;
    end if;
end process;

--%%%%%%%%%%%%% EX_MEM STAGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
EX_MEM: process(clk,rst)
begin
    if(rst = '1') then
        -- data
        PC_in_branch_ex_mem <= (others=>'0');
        ALU_result_ex_mem <= (others=>'0');
        dt2_ex_mem <= (others=>'0');
        wr_ex_mem <=  (others=>'0');
        -- control signals
        --WB
        rwr_ex_mem <= '0';
        memreg_ex_mem<= '0';
        -- MEM
        zero_ex_mem <= '0';
        branch_ex_mem <= '0';
        dmr_ex_mem <= '0';
        dmw_ex_mem <= '0';
    elsif(rising_edge(clk)) then 
        if(PCsrc = '0') then 
            -- data
            PC_in_branch_ex_mem <= PC_in_branch;
            ALU_result_ex_mem <= ALU_result;
            dt2_ex_mem <= dt2_id_ex;
            wr_ex_mem <=  wr_id_ex;
            -- control signals
            --WB
            rwr_ex_mem <= rwr_id_ex;
            memreg_ex_mem<= memreg_id_ex;
            -- MEM
            zero_ex_mem <= zero;
            branch_ex_mem <= branch_id_ex;
            dmr_ex_mem <= dmr_id_ex;
            dmw_ex_mem <= dmw_id_ex;
       else
           PC_in_branch_ex_mem <= (others=>'0');
           ALU_result_ex_mem <= (others=>'0');
           dt2_ex_mem <= (others=>'0');
           wr_ex_mem <=  (others=>'0');
           -- control signals
           --WB
           rwr_ex_mem <= '0';
           memreg_ex_mem<= '0';
           -- MEM
           zero_ex_mem <= '0';
           branch_ex_mem <= '0';
           dmr_ex_mem <= '0';
           dmw_ex_mem <= '0';
       end if;
    end if;
end process;
--%%%%%%%%%%%%% MEM_WB STAGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MEM_WB: process(clk,rst)
begin
    if(rst = '1') then
        ALU_result_mem_wb <= (others=>'0');
        wr_mem_wb <= (others=>'0');
        do_mem_wb <= (others=>'0');
        -- control signals -- WB
        rwr_mem_wb <= '0';
        memreg_mem_wb <= '0';
    elsif(rising_edge(clk)) then 
        -- data
        ALU_result_mem_wb <= ALU_result_ex_mem;
        wr_mem_wb <= wr_ex_mem;
        do_mem_wb <= do;
        -- control signals -- WB
        rwr_mem_wb <= rwr_ex_mem;
        memreg_mem_wb <= memreg_ex_mem;
    end if;
end process;
--%%%%%%%%%%%%% WB_ALU STAGE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
WB_ALU:process(clk,rst)
begin
    if(rst = '1') then
        wr_dt_wb_alu <= (others=>'0');
        wr_wb_alu <= (others=>'0');
        --control signal
        rwr_wb_alu <= '0';
    elsif(rising_edge(clk)) then 
        wr_dt_wb_alu <= wr_dt;
        wr_wb_alu <= wr_mem_wb;
        --control signal
        rwr_wb_alu <= rwr_mem_wb;
    end if;
end process;
--%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
--___________________________ updated
wr_dt <= ALU_result_mem_wb when memreg_mem_wb = '0' else
            do_mem_wb;
--__________

--%%%%%%%%%%%%%%%%%%%%__HAZARAD DECTECTION__%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load_add:process(dmr_id_ex,rd1,rd2,wr_id_ex)
begin
    if(dmr_id_ex = '1') then
        if(((wr_id_ex = rd1) or ((wr_id_ex =rd2)and (ALUsrc= '0'))) and (not (wr_id_ex ="00000"))) then
            stall <= '1';
        else
            stall <= '0';
        end if;
    else
        stall <= '0';
    end if;
end process;
en <= (not stall);
rs_rd1_rd2: process(stall,rwr_mem_wb,wr_mem_wb,rd2_id_ex,rd1_id_ex,ALUsrc_id_ex)
begin
    if((wr_ex_mem = rd1_id_ex) and (rwr_ex_mem = '1')) then
        forward_A <= "01";
    elsif((rwr_mem_wb = '1') and (wr_mem_wb=rd1_id_ex)) then
        forward_A <= "10";
    else
        forward_A <= "00";
    end if;
    
    if((wr_ex_mem=rd2_id_ex) and (rwr_ex_mem = '1') ) then
        forward_B <= "01";
    elsif((rwr_mem_wb = '1') and ((wr_mem_wb=rd2_id_ex) and (ALUsrc_id_ex= '0')) ) then
        forward_B <= "10";
    else
        forward_B <= "00";
    end if;
end process;

--%%%%%%%%%%%%%%%%%%%%%%__FORWARDING___%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
--EXECUTION STATE ____ Input to ALU ___ updated

B_forward <= dt2_id_ex when forward_B ="00" else
                ALU_result_ex_mem when forward_B ="01" else
                wr_dt when forward_B ="10" else
                wr_dt when forward_B ="11";
B <= B_forward  when ALUsrc_id_ex = '0' else
                        imm_id_ex;
                        
A <= dt1_id_ex when forward_A ="00" else
        ALU_result_ex_mem when forward_A ="01" else
        wr_dt when forward_A ="10" else
        wr_dt when forward_A ="11";
R4_out <= do4_out(7 downto 0);
end Behavioral;
--%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%____END______%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
