# ABDN-Kinematics-Project

Kinematics &amp; Dynamics Online Group 1 Design Project

![UI showing forward and reverse kinematics](robot_plot.png)


## The Report

All the files for generating the report are in the [report/](report/) directory.
In order to work on the report you will need the latex toolchain:

* [Mac - MacTex](https://www.tug.org/mactex/mactex-download.html) (be sure to put it on your `$PATH`)
* [Windows - TexLive](https://www.tug.org/texlive/)

And if working in [Visual Studio Code](https://code.visualstudio.com/), the
[LaTeX
Workshop](https://marketplace.visualstudio.com/items?itemName=James-Yu.latex-workshop)
extension is highly recommended.

### Building the report - LaTeX Workshop
* Save the `report.tex` file and the pdf will be automatically built

### Building the report - CLI
```shell
cd report 
pdflatex report.tex && bibtex report && pdflatex report.tex && pdflatex report.tex
```

Running `pdflatex` multiple times is required because ¯\_(ツ)_/¯