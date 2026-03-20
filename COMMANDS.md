# AT Commands

## Visible help anchor

- `AT+STEP?`

## Operational command

- `AT+STEP=ADD,<ALIAS>,<STEP>,<DIR>,<ENA|-1>`
- `AT+STEP=<ALIAS>,SPEED,<steps_s>`
- `AT+STEP=<ALIAS>,ACCEL,<steps_s2>`
- `AT+STEP=<ALIAS>,SPMM,<steps_per_mm>`
- `AT+STEP=<ALIAS>,MAX,<max_steps>`
- `AT+STEP=<ALIAS>,MAXMM,<max_mm>`
- `AT+STEP=<ALIAS>,MOVE,<delta>`
- `AT+STEP=<ALIAS>,MOVETO,<pos>`
- `AT+STEP=<ALIAS>,MOVEMM,<delta_mm>`
- `AT+STEP=<ALIAS>,GOTO,<pos_mm>`
- `AT+STEP=<ALIAS>,STOP`
- `AT+STEP=<ALIAS>,ESTOP`
- `AT+STEP=<ALIAS>,ENA,ON|OFF`

## Notes

- `MOVEMM` and `GOTO` require `SPMM` to be configured
- `MAX` and `MAXMM` define the absolute range `0..max`
- `STOP` requests deceleration
- `ESTOP` stops immediately

## Example

```text
AT+ESP=STEPPER,ENABLE,FALSE,TRUE
AT+STEP=ADD,X,17,18,19
AT+STEP=X,SPMM,91.43
AT+STEP=X,MAXMM,1200
AT+STEP=X,SPEED,20000
AT+STEP=X,ACCEL,30000
AT+STEP=X,MOVEMM,10
AT+STEP=X,GOTO,250
AT+STEP=X,STOP
```
