MODULE ArcWeldingModule
    CONST robtarget WeldStart:=[[400,0,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget WeldEnd:=[[600,100,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    
    VAR welddata WeldData;
    
    PROC ArcWeldMain()
        ! Initialize weld parameters
        WeldData.amperage := 200;
        WeldData.voltage := 22;
        WeldData.feedspeed := 8.5;
        
        ! Approach position
        MoveJ [[300,0,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v1000,z50,tWeldGun;
        
        ! Start arc and weld
        ArcLStart WeldStart,v100,fine,WeldData,\WeaveID:=1;
        ArcL WeldEnd,v100,fine,WeldData,\WeaveID:=1;
        ArcLEnd WeldEnd,v100,fine,WeldData,\WeaveID:=1,\CraterFillData:=CraterData;
        
        ! Retreat
        MoveJ [[300,0,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v1000,z50,tWeldGun;
    ENDPROC
ENDMODULE
