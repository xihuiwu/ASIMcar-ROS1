|Mode |Mode Name      |Denver 2 |Frequency |ARM A57 |Frequency |GPU Frequency|
|-----|:-------------:|:-------:|:--------:|:------:|:--------:|------------:|
|0	  |Max-N	        |2	      |2.0 GHz   |4       |2.0 GHz   1.30 Ghz      |
|1	  |Max-Q	        |0        |	 	       |4       |1.2 Ghz   |0.85 Ghz     |
|2	  |Max-P Core-All |2        |	1.4 GHz  |4	      |1.4 GHz	 |1.12 Ghz     |
|3	  |Max-P ARM	    |0        |	 	       |4	      |2.0 GHz	 |1.12 Ghz     |
|4	  |Max-P Denver   |1      	|2.0 GHz	 |1	      |2.0 GHz	 |1.12 Ghz     |

# Change mode:
sudo nvpmodel -m [mode]

# Find current mode:
sudo nvpmodel -q
