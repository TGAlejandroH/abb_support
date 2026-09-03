%%%
  VERSION:1
  LANGUAGE:ENGLISH
%%%

MODULE mRS999999_Stn1
    !---------------------------------------------------------------------------
    ! Weld-selector template - MONARC cell, STATION 1
    !
    ! Sample only. Shows the MONARC/ABB-Positioner idiom for:
    !   1. indexing the turntable 180 deg so stn 1 faces the robot
    !   2. taking the stn 1 tilt + chuck as the cell's two external axes
    !   3. moving robot / tilt / chuck, alone and together
    !
    ! Once STN1 is active this is a plain 2-axis positioner cell:
    !     extax.eax_b = ARM1   (logical axis 8)  - TILT
    !     extax.eax_c = PLATE1 (logical axis 9)  - ROTATE / CHUCK
    !     extax.eax_a / eax_d / eax_e / eax_f = 9E9  (unused)
    !
    ! CAUTION - the slots mean different motors while INTERCH is the active
    ! unit (5 motors share 3 logical axes):
    !     eax_b = INTERCH_PLATE1 (stn 1 chuck)
    !     eax_c = INTERCH_PLATE2 (stn 2 chuck)
    !     eax_d = INTERCH        (the 180 deg index axis)
    ! So the index angle is written to eax_d, never to eax_c.
    !
    ! Names used below are the real ones in this controller:
    !   IndexToStn1, ActStn1, DeactStn1, CalibIntch1  Irbp1Prc.sys / IrbpSetup.sys
    !   nInterchStn1 = -0.249999, nInterchStn2 = 179.901   IrbpSetup.sys
    !       (calibrated index positions - NOT exactly 0 / 180)
    !   sdInterch1, nTorqueTime, ldLoadStn1/2              Irbp1Data.sys
    !   wobj_Stn1 (coordinated) / wobj_Stn1_NoCoord        wobj_Database.sys
    !   tWeldGun                                           BE_User.sys
    !   jposSafe                                           Utility.sys
    !---------------------------------------------------------------------------

    LOCAL PERS speeddata vStn1Ax:=[10,10,5,20];

    ! Dummy taught points. extax = [eax_a, eax_b(tilt), eax_c(chuck), d, e, f]
    LOCAL CONST robtarget pS1_App:=[[960.27,53.69,-89.62],[0.704235,0.472417,0.0208334,0.529567],[-1,-1,-1,0],[9E+09,0,-22,9E+09,9E+09,9E+09]];
    LOCAL CONST robtarget pS1_A  :=[[960.21,-76.42,-109.08],[0.70423,0.472425,0.0208375,0.529566],[-1,-1,-1,0],[9E+09,0,-22,9E+09,9E+09,9E+09]];
    LOCAL CONST robtarget pS1_B  :=[[986.00,-110.25,-83.45],[0.691738,0.49066,0.00737006,0.529808],[-1,-1,-1,0],[9E+09,0,-22,9E+09,9E+09,9E+09]];
    ! same TCP pose, tilt/chuck moved -> robot and both station axes run together
    LOCAL CONST robtarget pS1_C  :=[[986.00,-110.25,-83.45],[0.691738,0.49066,0.00737006,0.529808],[-1,-1,-1,0],[9E+09,-20,45,9E+09,9E+09,9E+09]];
    LOCAL CONST robtarget pS1_Ret:=[[927.90,-60.40,-21.22],[0.400488,0.726508,0.549474,0.0993656],[-1,-1,2,0],[9E+09,-20,45,9E+09,9E+09,9E+09]];

    PROC RS999999_Stn1()
        VAR jointtarget jtCurrent;

        !--- 1. Nothing held, robot clear of the turntable -------------------
        DeactUnit STN1;
        DeactUnit STN2;
        MoveAbsJ jposSafe\NoEOffs,v1000,fine,tWeldGun;

        !--- 2. Index 180 deg: bring station 1 to the robot ------------------
        ! One call. It activates INTERCH, drives eax_d to the calibrated
        ! -0.249999 (stn 1 side) at sdInterch1, holds torque, deactivates.
        ! Robot must already be clear - the index does not check.
        IndexToStn1;
        ! NOTE: IndexToStn1 also pre-positions both chucks from the Production
        ! Manager part queue (GetNextPartAdv). With no part queued it drives
        ! them to 0 and posts a warning. If the weld selector does not feed the
        ! PM queue, use IndexStn1Direct below instead.

        !--- 3. Take the station-1 tilt + chuck -----------------------------
        ! ActStn1 = ActUnit STN1 + MechUnitLoad STN1,2,ldLoadStn1
        ActStn1;

        !--- 4. Station axes only - robot stands still -----------------------
        ! Read where we are, overwrite just the external axes, move there.
        jtCurrent:=CJointT();
        jtCurrent.extax.eax_b:=0;        ! tilt   ARM1   to 0 deg
        jtCurrent.extax.eax_c:=-22;      ! chuck  PLATE1 to -22 deg
        MoveAbsJ jtCurrent,vStn1Ax,fine,tool0;

        !--- 5. Robot only - table holds still -------------------------------
        ! wobj_Stn1 is coordinated (ufprog=FALSE): the object frame rides
        ! PLATE1, so these TCP positions stay correct at any chuck angle.
        MoveJ pS1_App,v600,z50,tWeldGun\WObj:=wobj_Stn1;
        MoveL pS1_A,v200,z10,tWeldGun\WObj:=wobj_Stn1;
        MoveL pS1_B,v200,fine,tWeldGun\WObj:=wobj_Stn1;

        !--- 6. Robot + tilt + chuck in one instruction ----------------------
        ! The extax fields of pS1_C differ from pS1_B, so all three interpolate
        ! together to arrive simultaneously. This is how a coordinated weld
        ! (ArcL) would be written - swap MoveL for ArcL and add seam/weld data.
        MoveL pS1_C,v100,fine,tWeldGun\WObj:=wobj_Stn1;

        !--- 7. Release and park --------------------------------------------
        ! The move before DeactUnit MUST end in a fine point (TRM limitation).
        MoveJ pS1_Ret,v600,fine,tWeldGun\WObj:=wobj_Stn1;
        DeactStn1;
        MoveAbsJ jposSafe\NoEOffs,v1000,z50,tWeldGun;
    ENDPROC

    !---------------------------------------------------------------------------
    ! Index to station 1 without the Production Manager part queue.
    ! Same sequence as ABB's IndexToStn1, minus GetNextPartAdv.
    !---------------------------------------------------------------------------
    PROC IndexStn1Direct()
        VAR jointtarget jtIdx;

        IF (NOT bInterchCalib1) CalibIntch1;
        IF (NOT IsMechUnitActive(INTERCH)) ActUnit INTERCH;
        MechUnitLoad INTERCH,2,ldLoadStn1;      ! stn 1 chuck load
        MechUnitLoad INTERCH,3,ldLoadStn2;      ! stn 2 chuck load

        jtIdx:=CJointT();
        jtIdx.extax.eax_b:=0;                   ! INTERCH_PLATE1 = stn 1 chuck
        jtIdx.extax.eax_c:=0;                   ! INTERCH_PLATE2 = stn 2 chuck
        jtIdx.extax.eax_d:=nInterchStn1;        ! the index itself (-0.249999)
        MoveAbsJ jtIdx,sdInterch1,fine,tool0;   ! fine - required before Deact

        WaitTime nTorqueTime;                   ! 0.4 s, let the clamp torque settle
        DeactUnit INTERCH;
        WaitUntil IsMechUnitActive(INTERCH)=FALSE;
    ENDPROC
ENDMODULE
