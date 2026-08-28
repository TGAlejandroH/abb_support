# RoboDK ABB Socket Driver: RW5/6 vs RW7



See files in the folder: resources\\Sample RAPID program for socket communication



Comparison of the two RoboDK RAPID driver modules:

* `RDK\_DriverSocket\_RW5\_6.mod` — RobotWare 5 and 6 (886 lines)
* `RDK\_DriverSocket\_RW7.modx` — RobotWare 7 (890 lines)

Both declare `MODULE RDK\_DriverSocket` and both require the **PC Interface** option.

\---

## Summary

The full diff is four hunks. **Only one is executable code.** The other three are comments.

|#|Location (RW5/6 → RW7)|Type|Change|
|-|-|-|-|
|1|line 10|Comment|Header text: "intended for RobotWare 5 and 6" → "intended for RobotWare 7"|
|2|233, 251 → 234, 253|Comment|Note added: RW7 does not support the `\\PStop` option on `SearchL`|
|3|\~367 → \~369|Comment|`! Break;` → `! RW7 replaced Break with DebugBreak` / `! DebugBreak;`|
|4|803 → 806–807|**Code**|`UnpackRawBytes ... \\ASCII` → `\\UTF8Encoding`|

\---

## The only functional change

```rapid
! RW5/6 — line 803
UnpackRawBytes bufferIn, bufferIn\_Index, str\_array \\ASCII:=array\_sz;

! RW7 — line 807
UnpackRawBytes bufferIn, bufferIn\_Index, str\_array \\UTF8Encoding:=array\_sz;
```

RW7 removed the `\\ASCII` switch on `UnpackRawBytes` and replaced it with `\\UTF8Encoding`.

For 7-bit ASCII the bytes on the wire are identical, so the RoboDK wire protocol did not
change. Only the RAPID keyword did.

\---

## The three comment-only changes

**`SearchL` / `\\PStop`** — RW7 dropped the `\\PStop` option. Both files already used
`\\SStop`, so the instruction is unchanged in all three occurrences:

```rapid
SearchL \\SStop, DI\_SearchL, rt\_c1, rt\_target, progSpeed, progTool, \\Wobj:=progWObj;
```

**`Break` / `DebugBreak`** — RW7 renamed `Break` to `DebugBreak`. In both files the call is
commented out and `Stop` is used instead, so there is no runtime difference.

**Header comment** — documentation only.

\---

## Socket functions are byte-identical

Every socket-related line matches exactly between the two files — same instructions, same
optional arguments, same order:

|Instruction|Usage in both files|
|-|-|
|`socketdev`|`LOCAL VAR socketdev server\_socket;` / `client\_socket`|
|`SocketCreate`|`SocketCreate server\_socket;`|
|`SocketBind`|`SocketBind server\_socket, SERVER\_IP, PORT;`|
|`SocketListen`|`SocketListen server\_socket;`|
|`SocketAccept`|`SocketAccept server\_socket, client\_socket \\ClientAddress:=client\_ip, \\Time:=WAIT\_MAX;`|
|`SocketReceive`|`SocketReceive client\_socket, \\RawData:=bufferIn, \\ReadNoOfBytes:=n, \\Time:=t;`|
|`SocketSend`|`SocketSend client\_socket, \\RawData:=bufferOut, \\NoOfBytes:=4;`|
|`SocketClose`|`SocketClose server\_socket;` / `client\_socket`|
|`PackRawBytes`|`\\IntX:=DINT` and `\\Float4` — unchanged|
|`UnpackRawBytes`|`\\IntX:=DINT`, `\\IntX:=USINT`, `\\Float4` — unchanged (only `\\ASCII` changed)|

**Conclusion:** the RAPID socket messaging API did not change between RobotWare 6 and 7.
RoboDK ships two files because of a string-encoding keyword plus two unrelated RAPID
instructions that changed in the same release — not because of anything socket-specific.

\---

## File extension

Not visible in the diff, but required by RW7/OmniCore:

|RW5/6|RW7|
|-|-|
|`.mod`|`.modx`|
|`.sys`|`.sysx`|
|`.prg`|`.pgfx`|

RobotStudio and the OmniCore controller expect the new extensions even when file content is
unchanged.

\---

## Porting checklist

For your own RAPID socket code moving from RW6 to RW7:

1. Search for `\\ASCII` on `PackRawBytes` / `UnpackRawBytes` → replace with `\\UTF8Encoding`.
2. Search for `\\PStop` on search instructions → replace with `\\SStop`.
3. Search for `Break` → replace with `DebugBreak`.
4. Rename `.mod` → `.modx`, `.sys` → `.sysx`.
5. Confirm PC Interface is licensed on the OmniCore key (option numbering differs from IRC5).
6. If the RAPID side acts as a **server** (`SocketBind` / `SocketListen` / `SocketAccept`),
add an inbound firewall rule on the OmniCore controller for the port. Outbound client
connections are generally not affected.
7. Confirm Multitasking is licensed if the socket loop runs in a background task.

The socket layer itself should port unchanged. Budget migration effort for I/O, safety
configuration, and motion instead.

