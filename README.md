# finite-element-analysis

[![python](https://img.shields.io/badge/python-3.12-blue.svg)](https://www.python.org/)
![os](https://img.shields.io/badge/os-ubuntu%20|%20macos%20|%20windows-blue.svg)
[![license](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/sandialabs/sibl#license)

[![codecov](https://codecov.io/gh/Lejeune-Lab-Graduate-Course-Materials/finite-element-analysis/graph/badge.svg?token=p5DMvJ6byO)](https://codecov.io/gh/Lejeune-Lab-Graduate-Course-Materials/finite-element-analysis)
[![tests](https://github.com/Lejeune-Lab-Graduate-Course-Materials/finite-element-analysis/actions/workflows/tests.yml/badge.svg)](https://github.com/Lejeune-Lab-Graduate-Course-Materials/finite-element-analysis/actions)

# High Level Overview

This is a high level overview of how the FEA code functions. Overall the code is comprised of two parts: Pre-processing where meshing and setting boundary conditions occur and calling the solver to solve the hyperelastic problem that was defined.

## Pre-processing

This step is about setting up the problem, first by meshing the geometry then by applying boundary conditions. The pre-processing functions are contained in ```pre_process``` and can only handle flat rectangular geometry.

### Meshing

To construct a rectangular mesh, call the ```generate_rect_mesh_2d``` function. This will automatically generate a mesh of a L x H rectangle with the specified element type and density. It accomplishes this by first subdividing the geometry into a grid of smaller rectangles called cells based on the density. The nodes are then first applied along the cell sides based on the element type. Next the elements are constructed inside the cells and assigned the relevant nodes. For triangular elements, the cells are divided along the bottom-left to top-right diagonal. For quadrilateral elements, the entire cell is allocated. After iterating through each cell, the mesh is returned as a series of points representing the nodes and a series of connections representing the elements.

### Boundary Conditions & Loads

Once this is accomplished, each face/boundary is defined using ```identify_rect_boundaries```. Each node along a face has the same x- or y-position depending on the face. This allows for selecting all nodes along a face based on their position. Once the face nodes are found, the face elements are found by selecting all elements who has an edge along the face in question. This is then repeated for all four faces: left, right, top, and bottom.

With the faces identified, boundary conditions can be applied. Fixed displacements are applied with ```assign_fixed_nodes_rect``` and loads are applied using ```assign_uniform_load_rect```. They both function similarly by assembling an array where the columns represent the node id, the dof id, and the displacement/force value. The main difference between the two functions is that ```assign_fixed_nodes_rect``` assigns each node the same displacement while ```assign_uniform_load_rect``` distributes the load across the face.

## Solver

Once the problem is fully defined, it is passed to a solver inside ```solver```. Currently the only solver is the ```hyperelastic``` solver. The solver peforms the following broad-level procedure: Starting with a reduced load and displacement, it calculates three things: A global stiffness matrix $K$ representing how the force changes with displacement, a force vector $F$ from the applied laods, and a residual vector $R$ that represnts the difference between the force calculated from the current displacement and the forces applied at the boundary. Once calculated, the vector $R$ is then used to update the displacement based on $K$. This repeats until the change in displacement is below a tolerance, indicating convergence. The load and displacements are then increase and the process is repeated until the boundary conditions are fully applied.

### Matrix Assembly

The main process is the matrix assembly taking place in the ```assembly``` module. This loop is performed mostly the same way across $K$, $F$, and $R$ where a for loop iterates over the elements inside the mesh. At each mesh a local $K$, $F$, or $R$ is calculated and then added to the global version. Local versions of $K$, $F$, and $R$ are calculated using the ```local_element``` module which handles anything that requires calculating on a local mesh. To assist, this module also calls ```discretization``` to provide information about element types such as node order, face location, etc. ```discretization``` is also called by ```assembly``` for the same reason.

### Updating Displacement

Once all the global matrices and vectors are calculated, fixed displacement conditions are applied by modifying the stiffness matrix $K$ and the residual matrix $R$. These are the final forms of $K$ and $R$.

If the displacements are accurate, then the residuals $R$ should be equal to zero. If they are not, they can be used to update the displacement using the Newton-Raphson method with $K$ acting as the Jacobian. Through this, a change in displacement is calculated and applied and the process is repeated from assembly.

### Convergence

If the change in displacement is below a specified tolerance, the the Newton-Raphson method has converged. Once this is achieved, the load is then increased and the process repeat again. The load is slowly increased over a series of load steps until the load reaches its original value. The displacement and forces are then returned as the results of the solver

## Process Flowchart and Dependency Diagram

Shown below is a more detailed diagram of the process involved in the FEA code. Each module is listed at the top and arrows show calls each module makes to one another.

![Process Chart](readme_img/FEA%20Diagram.drawio.png)

Below is the dependency chart for the FEA code.

![Dependency Chart](readme_img/FEA%20Dependency.drawio.png)

# Conda environment, install, and testing

Note: this is an extremely minimalist readme, but the code is highly documented and will get built out over the coures of assignment 3.

```bash
conda create --name me700-hw3-env python=3.12.9
```

```bash
conda activate me700-hw3-env
```

```bash
python --version
```

```bash
pip install --upgrade pip setuptools wheel
```

```bash
pip install -e .
```

```bash
pytest -v --cov=. --cov-report term-missing
```

