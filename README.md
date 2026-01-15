# UniCNet: Unified Cycle-Accurate Simulation for Composable Chiplet Network with Modular Design-Integration Workflow

UniCNet is a cycle-accurate simulator supporting effienct simulation for composable chiplet networks.



## Features
- Unified and accurate evaluation of entire composable chiplet network aligh with the design flow of chiplet-based architectures.
- Support key design features oriented towards multi-chiplet scenario, including composable topology, modular routing and heterogeneous router.
- Cycle-level chiplet protocal interface model, supporting UCIe protocol and verified it against RTL model.
- Enable multi-thread parallel simulation, achieving up to 4X speedup while maintaining the same accuracy with Booksim2.0.

## Simulation

### 1. Git Clone
```bash
git clone https://github.com/wangplin/CAL-UniCNet.git UniCNet
cd UniCNet
``` 

### 2. Install dependency
```bash
apt install bison
apt install flex
```

### 3. Build UniCNet
```bash
cd src
make -j$(nproc)
```

### 4. Run UniCNet

```bash
cd ../configs
./../src/booksim UniCNet.cfg
```

## Examples (configs/UniCNet.cfg)

### Active Interposer

```verilog
// active interposer
integration_file = integration_active_file;
interposer_config = interposer.cfg;
```

### Passive Interposer

```verilog
// passive interposer
integration_file = integration_passive_file;
routing_function_chiplet = chip_level_dor_mesh;
```

### UCIe Interface

```verilog
// ucie interface
// more parameters can be found in booksim_config.cpp
interface = ucie;

ucie_flit_size = 64;  
ucie_serdes_ratio = 4;
ucie_lane_width = 0.9;
ucie_error_rate = 0.1;
```

### Parallel Simulation

```verilog
enable_parallel = 1;
num_threads = 16;
```

## References

[1] Jiang, N., Becker, D.U., Michelogiannakis, G., Balfour, J., Towles, B., Shaw, D.E., Kim, J. and Dally, W.J., 2013, April. A detailed and flexible cycle-accurate network-on-chip simulator. In 2013 IEEE international symposium on performance analysis of systems and software (ISPASS) (pp. 86-96). IEEE.

[2] https://github.com/booksim/booksim2

## Citation

If you use UniCNet in your research, we would appreciate the following citation in any publications to which it has contributed:

```bibtex
@ARTICLE{11347548,
  author={Wang, Peilin and Wang, Mingyu and Ye, Zhirong and Lu, Tao and Yu, Zhiyi},
  journal={IEEE Computer Architecture Letters}, 
  title={UniCNet: Unified Cycle-Accurate Simulation for Composable Chiplet Network With Modular Design-Integration Workflow}, 
  year={2026},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/LCA.2026.3653809}}
```













