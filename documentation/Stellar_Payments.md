# Payment sequences #

### Activate routing: ###
- activateRoutingRoute.js
- Check password is good
- Check user balance
- Total up routing amounts for totalDeliveryCost + totalWorkstationCost + setting.networkFee
- If ok we continue ahead, else return status 400
- Create routingAccount with initial balance of 5
- Check balance is 5
- Send payment from user account with feeBump by networkAccount to networkAccount, total defined in setting.networkFee. Status 405 if fail
- Send payment from user account with feeBump by neworkAccount to routingAccount, to is the sum of totalDeliveryCost + totalWorkstationCost.  Status 404 if fail
- store networkTransaction href & activationTransaction in kpv for the step; return 14008 if not successful
- Update TotalWorkstationCosts = sum of all workstations
- Update TotalDeliveryCosts = sum of all deliveries
- Update actualFees to include transaction fees

### Worker loads workstation ###
- No payments if workstation is a buffer
- workerAction.js: workerAction.loadWorkstation
- If routingBalance is less than deliveryCost return 14006
- If there is more than one asset owner return 14007 (See https://github.com/mukmalone/AdvancedGoalManufacturing/issues/146)
- Send payment from routingAccount with feeBump by networkAccount to workerOwner.records[0].id
- Store deliveryTransaction in kpv for the step
- Update actualWorkerCost = actualWorkerCost + deliveryCost
- Update actualFees to include transaction fees

### Workstation completes ###
- No payments if in auto workstation cycling mode
- workstationRoute.js
- if routingBalance is less thank workstationCost return 14006
- if there is more than one workstation owner return 14007
- Send payment from routingAccount with feeBump by networkAccount to workstationOwner.records[0].id
- Store workstationTransaction in kpv for the step
- Update actualWorkstationCost = actualWorstationCost + workstationCost
- Update actualFees to include transaction fees

### Outfeed Buffer ###
- activeRouting becomes a completedRouting
- if the routing balance is greater than the initial balance of the routingAccount refund is required
- refund and merge
- otherwise just merge
- Update actualFees to include transaction fees

