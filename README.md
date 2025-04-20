# Example STM32 project for the Template Baord with StmEpic and C++

## Steps to build the project

1. Install required tools and setup environment configuration by following **_Geting Started_** guide on StmEpic [website](https://stmepic.d3lab.dev/md_docs_pages_geting_started.html).

2. Clone the repository to your project directory .

```bash
git clone git@github.com:Wust-Sat/TB_embeded_base_project.git
cd TB_embeded_base_project
```

3. Run init script to setup the project.

```bash
./init_repo.sh
```

4. Build the project.

```bash
./compile.sh
```

5. Add your code in **src** directory.

### After generating code in CubeMX

1. Remove Tiemr callback function from **main.c** file.

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  ...
}
```

2. Run the command

```bash
./compile.sh
```
