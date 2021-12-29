# Payment sequences #

### Activate routing: ###
- Check password is good
- Check user balance
- Total up routing amounts for totalDeliveryCost + totalWorkstationCost + setting.networkFee
- If ok we continue ahead, else return status 400
- Create routingAccount with initial balance of 5
- Check balance is 5
- Send payment from user account with feeBump by networkAccount to networkAccount, total defined in setting.networkFee. Status 405 if fail
- Send payment from user account with feeBump by neworkAccount to routingAccount, to is the sum of totalDeliveryCost + totalWorkstationCost.  Status 404 if fail
- store networkTransaction href & activationTransaction in kpv for the step; return 14008 if not successful

### Worker loads workstation ###
- No payments if workstation is a buffer
- If routingBalance si less than deliveryCost return 14006
- If there is more than one asset owner return 14007 (See https://github.com/mukmalone/AdvancedGoalManufacturing/issues/146)
- Send payment from routingAccount with feeBump by networkAccount to workerAowner.records[0].id
- Store deliveryTransaction in kpv for the step
- Update actualWorkerCost = actualWorkerCost + deliveryCost
