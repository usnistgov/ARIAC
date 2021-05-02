Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

- [Wiki | Documentation | Scoring](#wiki--documentation--scoring)
  - [Stability](#stability)
  - [Cost metrics](#cost-metrics)
    - [Team System Cost (TC)](#team-system-cost-tc)
    - [Baseline System Cost (BC)](#baseline-system-cost-bc)
    - [Cost Factor (**CF)**](#cost-factor-cf)
  - [Performance metrics](#performance-metrics)
    - [Shipment Completion Score (SCS)](#shipment-completion-score-scs)
      - [Kitting](#kitting)
      - [Assembly](#assembly)
    - [Order Completion Score (OCS)](#order-completion-score-ocs)
    - [Order Time (OT)](#order-time-ot)
    - [Average Time (AT)](#average-time-at)
    - [Efficiency Factor (**EF**)](#efficiency-factor-ef)
    - [Collision (collision)](#collision-collision)
  - [Trial Score (**TS**)](#trial-score-ts)

# Wiki | Documentation | Scoring

- Scores will be made up of: 
  1. An automatically calculated component.
  2. A score based on judges.
- The automatically calculated component for each trial is a combination of cost and performance metrics.
- A team's final score will be calculated as the sum of scores for each of the competition trials.
- See [challenge.gov](https://www.challenge.gov/challenge/ariac/) for details on the judges' scoring (scroll to Judging Criteria).
- **Note** that only automated metrics will be used during the qualification rounds of the competition (no judges' scoring).

## Stability

- As outlined in the [update policy](update_policy.md), changes to the scoring metrics may be made between rounds of the competition at the discretion of the competition controllers.
- Qualifying teams will be made aware of the final scoring parameters in advance of the Finals of the competition.

## Cost metrics

- The cost metrics are calculated from a team's system setup.
- Since the same system setup is used for **all trials**, this value is constant for a competitor for all trials.
- The value in **bold** is used in the final score calculation.

### Team System Cost (TC)

- Cost of a competitor's system (sum of the cost of sensors used).
  - $600 Camera mounted on the robot. **\*\*New this year\*\***
  - $500 for each logical camera used.
  - $300 for each RGBD camera used.
  - $200 for each depth camera used.
  - $100 for each other sensor used.
  
### Baseline System Cost (BC)

A constant cost amount, set by NIST.

### Cost Factor (**CF)**

A comparison of <img src="https://render.githubusercontent.com/render/math?math=BC"> to <img src="https://render.githubusercontent.com/render/math?math=TC">.
   - Competitors that have the same <img src="https://render.githubusercontent.com/render/math?math=TC"> will have <img src="https://render.githubusercontent.com/render/math?math=\mathbf{CF} = 1">
   - Competitors with a <img src="https://render.githubusercontent.com/render/math?math=TC"> lower than <img src="https://render.githubusercontent.com/render/math?math=BC"> are rewarded by having  <img src="https://render.githubusercontent.com/render/math?math=\mathbf{CF} > 1"> 
   - Teams with a <img src="https://render.githubusercontent.com/render/math?math=TC"> higher than <img src="https://render.githubusercontent.com/render/math?math=BC"> are penalized by having  <img src="https://render.githubusercontent.com/render/math?math=\mathbf{CF} < 1">
   - <img src="https://render.githubusercontent.com/render/math?math=\mathbf{CF} = \frac{BC}{TC}\,\text{where}\,BC = 8200">
   <!-- - **CF** = (BC / TC) where BC = 10000 -->

   <!--* `CF = (BC / TC)` where `BC = 1700`-->

## Performance metrics

- Performance metrics cover both *completion* and *efficiency* and they are calculated for each trial separately.
  - *Completion* captures the quality of the shipments submitted.
  - *Efficiency* captures the responsiveness in fulfilling orders.
- There are up to two orders requested during each trial, each composed of up to two shipments.
- The following values are calculated for each order requested in a trial.
- The values in **bold** will be used in the final score calculation for a trial.

<!-- **Note** that products must be placed onto the base of the shipping box to count for scoring, not on top of other products. -->
### Shipment Completion Score (SCS)

#### Kitting

- <img src="https://render.githubusercontent.com/render/math?math=\text{For each product}\,i\,\text{in a shipment of}\, n\,\text{products,}\, SCS=(\sum_{i=1}^{n}PCT_{i}%2BPCC_{i}%2BPCP_{i}%2Bbonus)\times destination,\,\text{where:}">
  
  - <img src="https://render.githubusercontent.com/render/math?math=\text{Product correct type (}PCT_i\text{): 1pt if product}\, i\, \text{ is of the correct type and touching the tray.}"> 
  
    - *Example*: If the robot places `assembly_pump_blue` in the tray while `assembly_pump_red` was requested in the order, then 1pt is awarded since the type (`pump`) matches.
  - <img src="https://render.githubusercontent.com/render/math?math=\text{Product correct color (}PCC_i\text{): 1pt if product}\,i\, \text{ has the correct color.}"> 

    - **NOTE**: If the order requests `assembly_pump_blue` but `assembly_battery_blue` is placed in the tray by the robot, then no point is awarded. Even though these two products have the same color (`blue`), they are not of the same type (`pump` Vs. `battery`). ***The type of a product has a higher priority than the product's color in ARIAC***.

  - <img src="https://render.githubusercontent.com/render/math?math=\text{Product correct pose (}PCP_i\text{): 1pt if product}\,i\,\text{is in the correct pose in the tray.}"> 
  
    - The location must be within 3 cm of the target and the orientation must be within 0.1 rad.
  
  - <img src="https://render.githubusercontent.com/render/math?math=\text{All products bonus (}bonus\text{): Points totaling the number of products}\, n\,\text{in the shipment}.">
  
    - <img src="https://render.githubusercontent.com/render/math?math=\text{For}\,n\,\text{products in the shipment,}\,bonus=n\, \text{iff the following is true for each product}\, i :"> 
  
      - <img src="https://render.githubusercontent.com/render/math?math=PCT_{i}=1\wedge PCC_{i}=1\wedge PCP_{i}=1\wedge\,\text{product}\,i\, \text{is not faulty}">.
  
  - <img src="https://render.githubusercontent.com/render/math?math=\text{Destination (}destination\text{): Wrong destinations will set the score for the shipment to 0.}">

    - <img src="https://render.githubusercontent.com/render/math?math=\text{For kitting shipments, }\,destination=dest_{agv}\times dest_{as}">
      
      - <img src="https://render.githubusercontent.com/render/math?math=\text{If a kitting shipment is built on the wrong AGV then}\,dest_{agv}=0\,\text{otherwise,}\,dest_{agv}=1">
      - <img src="https://render.githubusercontent.com/render/math?math=\text{If a kitting shipment is delivered to the wrong assembly station then}\,dest_{as}=0,\text{otherwise,}\,dest_{as}=1">
 
- Test case: The instructions below will allow you to ship an AGV to an assembly station so you can visualize the score breakdown for a kitting shipment.
  - [sample_kitting_scoring_test.yaml](../../nist_gear/config/trial_config/sample_kitting_scoring_test.yaml) consists of AGV2 ready to be delivered. Products located on this AGV match the products specified in the order.
    - 1. Modify [sample_environment.launch](../../nist_gear/launch/sample_environment.launch) to include `sample_kitting_scoring_test.yaml`
    - 2. `roslaunch nist_gear sample_environment.launch`
    - 3. `rosservice call /ariac/start_competition`
    - 4. `rosservice  call /ariac/agv2/submit_shipment "as1" "order_0_kitting_shipment_0"`

```text
Score breakdown:
[game_score]
...total game score: [10]
...total process time: [3.501]
...arms collision?: [0]
   [order score]
   ...order ID [order_0]
   ...total order score: [10]
   ...completion score: [10]
   ...time taken: [3.499]
   ...priority: [1]
      [kitting score]
      ...shipment type: [order_0_kitting_shipment_0]
      ...completion score: [10]
      ...complete: [false]
      ...submitted: [true]
      ...used correct agv: [true]
      ...sent to correct station: [true]
      ...product type presence score: [3]
      ...product color presence score: [3]
      ...product pose score: [3]
      ...all products bonus: [1]
```
#### Assembly

- <img src="https://render.githubusercontent.com/render/math?math=\text{For each product}\,i\,\text{in a shipment of} n \text{products},\, SCS=(\sum_{i=1}^{n}success_{i}%2Bcolor_{i}%2Bbonus)\times destination,\, \text{where:}">

  - <img src="https://render.githubusercontent.com/render/math?math=success_{i}=2\text{pts for each product successfully assembled.}">

    - These 2pts are awarded iff products: 
      - 1. have the correct type (e.g., `assembly_battery`)
      - 2. inserted in the correct pose
      - 3. not faulty

    - **NOTE**: Partial points are not awarded in assembly (either 0pt or 2pts for each product).
  
  - <img src="https://render.githubusercontent.com/render/math?math=color_{i}=1\text{pt for each product}\, i\,\text{if}\,success_{i}=2\,\text{AND the color is correct.}">
    
    - There is an extra point available for each product if it is the correct color e.g., `assembly_battery_blue`. 
    - **NOTE**: If the product is not of the correct type **AND** not in the correct pose **AND** not faulty then this extra point is not awarded, even if the product is of the correct color.
  
  - <img src="https://render.githubusercontent.com/render/math?math=bonus = 4\times n\,\text{for}\,n\,\text{products in the shipment iff}\, success_{i}=2\,\text{AND}\,color_{i}=1\,\text{for each product}\,i\,\text{in the shipment. Otherwise}\,bonus = 0.">
  
    - An all-products bonus is awarded to make a maximum of 4 times the number of products being inserted into the assembly (so for the ventilator briefcase design, the bonus would be a maximum of 16pts for a fully completed ventilator of 4 products).
    - **NOTE**: The all-products bonus is not associated to individual products but for the whole shipment. To be awarded the bonus all the products in the shipment have to be of the correct type **AND** placed in the correct pose **AND** not be faulty **AND** have the correct color.
   - <img src="https://render.githubusercontent.com/render/math?math=destination=0\,\text{if the briefcase was assembled  at the wrong assembly station, otherwise}\,destination=1.">

### Order Completion Score (OCS)

<img src="https://render.githubusercontent.com/render/math?math=OCS=\sum_{n=1}^{2} SCS_{n}">

The order completion score is the sum of the shipment completion score for each shipment  <img src="https://render.githubusercontent.com/render/math?math=n"> in the order.

### Order Time (OT)

The time taken to complete the order, measured from when the order was first requested. If there are 2 orders that need to be completed, then <img src="https://render.githubusercontent.com/render/math?math=OT_1"> and <img src="https://render.githubusercontent.com/render/math?math=OT_2"> will be computed.

### Average Time (AT)

<img src="https://render.githubusercontent.com/render/math?math=\text{Average time for all competitors to complete the order. If there are 2 orders that need to be completed then}\,AT_1\,\text{and}\,AT_2\,\text{will be computed}">

<!-- Average time for all competitors to complete the order. If there are 2 orders that need to be completed then <img src="https://render.githubusercontent.com/render/math?math=AT_1"> and <img src="https://render.githubusercontent.com/render/math?math=AT_2"> will be computed. -->

### Efficiency Factor (**EF**)

- <img src="https://render.githubusercontent.com/render/math?math=EF=\frac{AT}{OT}">

  - <img src="https://render.githubusercontent.com/render/math?math=\text{A comparison of a competitor's order time compared to all other competitors.}">

  - <img src="https://render.githubusercontent.com/render/math?math=\text{If there are 2 orders that need to be completed then}\,EF_1=\frac{AT_1}{OT_1}\,\text{and}\,EF_2=\frac{AT_2}{OT_2}">

### Collision (collision)

- <img src="https://render.githubusercontent.com/render/math?math=collision=0\,\text{in the following cases:}">
  
  - The kitting and the assembly robots collide with each other.
  - The arm on the assembly robot collides with the robot torso (including the torso tray).
- <img src="https://render.githubusercontent.com/render/math?math=\text{Otherwise,}\,collision=1">

## Trial Score (**TS**)

- <img src="https://render.githubusercontent.com/render/math?math=\text{The final trial score}\,\mathbf{TS}\,\text{for a trial is calculated as follows:}">
  
  - <img src="https://render.githubusercontent.com/render/math?math=\let \mathbf{CS_j}, \mathbf{EF_j}\,\text{be the values for the}\,\mathbf{j^{th}}\,\text{order in a trial,}\, \mathbf{j=1,2}">
  - <img src="https://render.githubusercontent.com/render/math?math=\let \mathbf{h=3}\,\text{as a high-priority factor that encourages quick response to the second (higher priority) order that is issued.}">
  -  <img src="https://render.githubusercontent.com/render/math?math=penalty:\,\text{1pt for each part dropped on the floor (intentionally or not).}">
  
     -  <img src="https://render.githubusercontent.com/render/math?math=\text{Competitors should use the faulty part collector to discard faulty products.}">

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{TS} = (collision \times (\mathbf{CF} \times avg(\sum_{j=1}^{2}\mathbf{CS_j})%2B\mathbf{EF_1} \times \mathbf{CS_1}%2B\mathbf{EF_2}\times \mathbf{CS_2}\times \mathbf{h})) - penalty">

<!-- ```
TS = COL * (
           CF * average(CS)
           + EF1 * CS1
           + EF2 * CS2 * h
           )
``` -->

-------------------------------------------------

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
