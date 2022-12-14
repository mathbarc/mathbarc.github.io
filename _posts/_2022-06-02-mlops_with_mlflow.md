---
title:  "MLOps with MLFlow"
author: matheus
date:   2022-06-02 10:00:00 -0300
categories: [DevOps, MLOps]
tags: [machine learning, devops]
pin: false
---

<div>
<img src="{{site.baseurl}}/github.png" width=20px height=20px /> <a href="https://github.com/mathbarc/mlops_with_mlflow">mathbarc/mlops_with_mlflow</a>
</div>


# What is MLOps?

![MLOps Model]({{site.baseurl}}/mlops_with_mlflow/mlops.png "MLOps Model")

MLOps is a set of practices used to reduce the distance between the creation of Machine Learning models and the final user of this model. This concept is based on the DevOps model but with additional steps for Machine Learning development:

- Capture and organization of data, the most important intake of machine learning approachs;
- Selection and training of models compatible with patterns which extraction from dataset is desired;
- Validation of models with data different then used in training.

These three steps are included as a extension of the development fase but generally on projects those steps occur in parellel, since the time necessary for maturing a machine learning model and the code base are different.

# Different components of MLOps process

{% include image.html url="/mlops_with_mlflow/mlops_components.png" description="Extracted from: <a href='https://databricks.com/glossary/mlops'>databricks.com</a>" %}


With aim of reducing disparities between code and ML models, the following steps are adopted into the development phase:

- **Exploratory data analysis:** In this step data scientists analyse the dataset to comprehand the problem and acess the viability of the solution;
- **Data preparation:** At this point the data is cleaned, organized and annotated;
- **Model training:** The organized dataset is divided into a training set and test set. Different models are created and trained on the data to extract its patterns and aftwards the test set is used to assure generalization of the models;
- **Model serving:** When a model reaches the quality standard desired it is moved to production where it will be used to infer new data;
- **Monitoring:** To garantee that the model keeps making decisions on the same quality desired on the training step, it is necessary to log results data and setup a feedback process from the users of this model. New data that performs badly is separated to integrate the dataset for new model training iterations.

# Maturity levels of MLOps processes

## Manual

{% include image.html url="/mlops_with_mlflow/manual_process.png" description="Extracted from: <a href='https://cloud.google.com/architecture/mlops-continuous-delivery-and-automation-pipelines-in-machine-learning
'>cloud.google.com</a>" %}

The manual process consists in creating each step of a Machine Learning Pipeline through scripts or notebooks that are manually started after each step of the pipeline. This generally is the first process implemented at the start of a project or in its Proof of Concept phase due to the low difficulty of creation.

Como primeiro pipeline temos aqui o Processo Manual. Nele a an??lise, prepara????o, treinamento e valida????o s??o todos realizados de forma manual, muitas vezes em scritps separados ou c??lulas de um notebook que s??o chamados manualmente nesta sequ??ncia para realizar o pipeline de treinamento de modelos.
Neste tipo de processo existe uma desconex??o entre o time de data science que implementa o modelo e os engenheiros de software que desenvolvem o servi??o que utilizar?? o modelo.

Apesar de seus problemas, esse processo n??o necessita de um ambiente estruturado para ser implantado, como ?? o caso de PoCs ou projetos cuja a viabilidade ainda n??o foi confirmada.

## ML Pipeline Automation

{% include image.html url="/mlops_with_mlflow/automated_ml_pipeline.png" description="Extracted from: <a href='https://cloud.google.com/architecture/mlops-continuous-delivery-and-automation-pipelines-in-machine-learning
'>cloud.google.com</a>" %}

Quando movemos a cria????o de um processo automatizado de treinamento chegamos ao segundo est??gio. Aqui temos uma pipeline automatizada para o processo o que permite execu????es mais r??pidas de experimentos. Outro grande ganho desta etapa ?? a cria????o de uma simetria entre o que ?? utilizado em todos os ambientes, com o pipeline automatizado ?? poss??vel replicar no ambiente de produ????o o mesmo ambiente utilizado para desenvolvimento dos modelos de ML. Essa possibilidade tr??s uma oportunidade: a entrega cont??nua de modelos. Novas melhorias e aprimoramentos assim que validados podem ser movidos para produ????o com maior facilidade.
Este est??gio ?? recomendado para produtos que ir??o come??ar a serem utilizados por uma user base restrita.

## ML Pipeline Automation with CI/CD

{% include image.html url="/mlops_with_mlflow/automated_cicd_pipeline.png" description="Extracted from: <a href='https://cloud.google.com/architecture/mlops-continuous-delivery-and-automation-pipelines-in-machine-learning
'>cloud.google.com</a>" %}

Nesse ultimo est??gio, o pipeline automatizado de machine learning possui testes e empacotamento automatizados via CI e entregas nos diversos ambientes do produto automatizados com praticas de CD, diminuindo ainda mais a dist??ncia entre o modelo e o contexto no qual ele ser?? utilizado. Nessa fase do produto estamos lidando com equipes maiores, maior complexidade e uma user base maior tamb??m.


# Using MLFlow

## Services Necessary

### Setting up Minio

### Setting up MLFlow

## Training a model

### Dataset used

### Model trained

### Managing Experiments on MLFlow

### Deploying a model

{% include youtube.html url="https://www.youtube.com/embed/G7300HZEiOo" %}

{% include signature.html %}

