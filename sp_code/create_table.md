```mermaid
flowchart TD
    id1((exec_state))-->id2{INIT}
    id1-->id2{WAIT_TARGET}
    id1-->id2{GEN_NEW_TRAJ}
    id1-->id2{EXEC_TRAJ}
    id1-->id2{REPLAN_TRAJ}
    style id1 fill:#f9f,stroke:#333,stroke-width:4px
    style id2 fill:#bbf,stroke:#f66,stroke-width:2px,color:#fff,stroke-dasharray: 5 5
```
    
```mermaid
flowchart LR
    id1(Start)-->id2(Stop)
    style id1 fill:#f9f,stroke:#333,stroke-width:4px
    style id2 fill:#bbf,stroke:#f66,stroke-width:2px,color:#fff,stroke-dasharray: 5 5
```
    
