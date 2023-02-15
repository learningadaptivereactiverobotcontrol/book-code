# errorbar_groups
ERRORBAR_GROUPS produces customizable grouped bar plots with overlaid error bars.

At its most basic, this function produces bar plots similar to those obtained using MATLAB's BAR(Y,'grouped') function call, and then overlays error bars onto the corresponding bars.

ERRORBAR_GROUPS allows customizing the plot in several ways. For instance, both the width of the bars themselves and that of the error bars can be adjusted. The function allows asymmetric values for the lower and upper bounds of the error bars. The colors of the bars and error bars can also be customized. ERRORBAR_GROUPS allows transmitting optional input property-value pairs to both the BAR and ERRORBAR functions, making it quite versatile.