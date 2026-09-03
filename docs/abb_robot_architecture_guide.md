# ABB RAPID Program Architecture: Dynamic Module Migration Guide (FANUC to ABB)

This document provides a comprehensive guide for translating a dynamic, multi-program FANUC architecture utilizing KAREL subroutines and socket communication into the **ABB RAPID** framework.

---

## 1. High-Level Architectural Metaphor
* **FANUC (The Loose Recipe Box):** Each program lives as an isolated standalone file (`.ls` / `.tp`). Global registers (`R`, `PR`, `SR`) are referenced uniformly across files, and communication or complex logic is outsourced to separate compiled binary KAREL files (`.pc`).
* **ABB (The Modular Binder):** Code is encapsulated within **Modules** (`.mod` files). A module is a structured text file containing its own local or global data variables (`VAR`), persistent data (`PERS`), and executable routines (`PROC` or `FUNC`). Rather than smashing everything into one file, you distribute logic across separate modules that share data natively.

---

## 2. Structural Breakdown

To replicate the setup where a Main program calls varying *N* programs that leverage *Y* shared subroutines and active socket streams, the system is split into two primary components:

### A. The Core Module (`MainServer.mod`)
* **Role:** Permanently resides in the controller's memory. It handles initialization, manages the socket server life cycle, and acts as the supervisor that dynamically routes execution.
* **Scope:** Hosts **`GLOBAL`** variables—specifically the socket connection descriptors—making them accessible to any other module loaded into the system memory.

### B. The Dynamic Modules (`Prog1.mod`, `Prog2.mod`, ..., `ProgN.mod`)
* **Role:** Houses the application-specific paths or sequences.
* **Flexibility:** Each *N* program lives in its own independent `.mod` file on the controller disk. They can be added, updated, or deleted without affecting or modifying the Core Module.

---

## 3. The Dynamic Workflow & Code Implementation

### Core Module Implementation
This module opens the socket, receives the string payload designating which program to execute, and triggers execution sequentially via **Late Binding** (the equivalent to FANUC's `CALL SR[x]`).

```rapid
MODULE MainServer
    ! GLOBAL variables are shared across ALL modules in the task image
    GLOBAL VAR socketdev server_socket;
    GLOBAL VAR socketdev client_socket;
    LOCAL VAR string program_name;

    PROC main()
        WHILE TRUE DO
            ! 1. Initialize and bind the socket server
            SocketCreate server_socket;
            SocketBind server_socket, "192.168.1.10", 5000; ! Controller IP & Port
            SocketListen server_socket;
            
            ! 2. Block execution until the PC client connects
            SocketAccept server_socket, client_socket;
            
            ! 3. Receive the target program string from the PC
            SocketReceive client_socket \Str:=program_name;
            
            ! 4. Execute the dynamic program via Late Binding
            ! The socket connection is left OPEN. The called routine inherits it.
            TRY
                ! The % delimiters instruct RAPID to evaluate the string as a routine name
                % program_name %;
            DEFAULT
                ! Emergency fallback context if the PC transmits an invalid routine name
                TPWrite "Error: Routine " + program_name + " not found in memory!";
                CleanUpSockets;
            ENDTRY
            
            ! The loop pauses while the dynamic routine runs. 
            ! Once the dynamic routine finishes and terminates, execution returns here.
        ENDWHILE
    ENDPROC

    PROC CleanUpSockets()
        SocketClose client_socket;
        SocketClose server_socket;
    ENDPROC
ENDMODULE
```

### Dynamic Module Implementation (`Prog1.mod`)
This independent file utilizes the open socket stream established by the Core Module, executes its unique cycle, and assumes responsibility for closing the socket connection upon completion.

```rapid
MODULE Prog1
    PROC RoutineInProg1()
        ! The client_socket connection is already open and ready for network I/O
        SocketSend client_socket \Str:="ROBOT_MOVING_TO_PICK";
        MoveL pPick, v500, z10, tGripper;
        
        SocketSend client_socket \Str:="PICK_COMPLETE";
        MoveL pPlace, v500, z10, tGripper;
        
        ! 5. Close the sockets at the terminal phase of the sequence
        SocketClose client_socket;
        SocketClose server_socket;
        
    ERROR
        ! Robust exception handling: ensures the socket drops cleanly if a fault occurs
        SocketClose client_socket;
        SocketClose server_socket;
        TRaise; ! Escalate the exception back up to the Core Module loop
    ENDPROC
ENDMODULE
```

---

## 4. Operational File Management (The ABB Way)

To add, update, or remove any of your *N* programs without altering your Core Module, use one of the two strategies below:

### Method 1: Automated File Interrogations (Dynamic Loading)
Instead of keeping all programs loaded in the active memory footprint, your Core Module can inspect the disk directory, load the file dynamically, run it, and immediately discard it to free memory.

```rapid
PROC ExecuteWithDiskLoading(string target_prog)
    ! 1. Dynamically mount the module file from the disk into memory
    Load "HOME:/YourFolder/" + target_prog + ".mod";
    
    ! 2. Execute via Late Binding
    % target_prog %;
    
    ! 3. Unload from active memory to allow future disk overwrites
    Unload "HOME:/YourFolder/" + target_prog + ".mod";
ENDPROC
```
* **Adding/Removing:** File transfers are handled via FTP or USB directly into the controller's storage directory (`HOME:/YourFolder/`). No code editing or controller restarts required.

### Method 2: Monolithic Active Memory (Static Loading)
All *N* module files are permanently loaded into the active task image. 
* **Updating/Removing:** If you need to revise `Prog3.mod`, navigate to the FlexPendant's Program Editor, select **Unload Module** for `Prog3`, replace the file on the flash memory, and select **Load Module**. The remaining `MainServer` logic remains continuously online and active.

---

## 5. Direct Paradigm Summary

| Architectural Action | FANUC Framework | ABB RAPID Framework |
| :--- | :--- | :--- |
| **File Format** | Standalone `.ls` or `.tp` files | Self-contained `.mod` text files |
| **Program Deployment** | Transfer `NEW.ls` to controller memory | Transfer `NEW.mod` to controller disk and invoke via `Load` |
| **Subroutine Execution** | `CALL KARELSUB` (Global system space) | `KARELSUB;` (Natively handled via global `PROC`/`FUNC`) |
| **Variable Program Routing** | `CALL SR[R[1]]` | `% StringVariable %;` (Native Late Binding) |
| **Network Data Scope** | KAREL localized descriptor sharing | `GLOBAL VAR socketdev` (Uniformly visible pipe) |