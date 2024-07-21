# Tutoriels divers
## Sommaire 
1. [Créer une intégration continue sur Gitlab](#créer-une-intégration-continue-sur-gitlab) 
2. [Compiler notre projet dans la pipeline](#compiler-notre-projet-dans-la-pipeline)
3. [Créer un package debian à partir d'un package ROS2 et l'installer sur son ordinateur](#créer-un-package-debian-à-partir-dun-package-ros2-et-linstaller-sur-son-ordinateur)
4. [La même chose dans le pipeline](#la-m%C3%AAme-chose-dans-le-pipeline)
5. [Pouvoir installer des packages depuis le serveur Nexus](#pouvoir-installer-des-packages-depuis-le-serveur-nexus)


## Créer une intégration continue sur Gitlab

### Créer un fichier .gitlab-ci.yml à la racine du projet
Le mieux c'est d'utiliser l'interface de Gitlab : aller dans Build > Pipeline Editor > Configure pipeline.
Il crée un fichier prérempli qui se présente ainsi : 
```yaml
# This file is a template, and might need editing before it works on your project.
# This is a sample GitLab CI/CD configuration file that should run without any modifications.
# It demonstrates a basic 3 stage CI/CD pipeline. Instead of real tests or scripts,
# it uses echo commands to simulate the pipeline execution.
#
# A pipeline is composed of independent jobs that run scripts, grouped into stages.
# Stages run in sequential order, but jobs within stages run in parallel.
#
# For more information, see: https://docs.gitlab.com/ee/ci/yaml/index.html#stages
#
# You can copy and paste this template into a new `.gitlab-ci.yml` file.
# You should not add this template to an existing `.gitlab-ci.yml` file by using the `include:` keyword.
#
# To contribute improvements to CI/CD templates, please follow the Development guide at:
# https://docs.gitlab.com/ee/development/cicd/templates.html
# This specific template is located at:
# https://gitlab.com/gitlab-org/gitlab/-/blob/master/lib/gitlab/ci/templates/Getting-Started.gitlab-ci.yml

stages:          # List of stages for jobs, and their order of execution
  - build
  - test
  - deploy

build-job:       # This job runs in the build stage, which runs first.
  stage: build
  script:
    - echo "Compiling the code..."
    - echo "Compile complete."

unit-test-job:   # This job runs in the test stage.
  stage: test    # It only starts when the job in the build stage completes successfully.
  script:
    - echo "Running unit tests... This will take about 60 seconds."
    - sleep 60
    - echo "Code coverage is 90%"

lint-test-job:   # This job also runs in the test stage.
  stage: test    # It can run at the same time as unit-test-job (in parallel).
  script:
    - echo "Linting code... This will take about 10 seconds."
    - sleep 10
    - echo "No lint issues found."

deploy-job:      # This job runs in the deploy stage.
  stage: deploy  # It only runs when *both* jobs in the test stage complete successfully.
  environment: production
  script:
    - echo "Deploying application..."
    - echo "Application successfully deployed."
```

Les ```stages``` ce sont les grandes étapes de la pipeline. 
On définit ensuite un nombre de ```jobs``` auxquels on associe une étape pour définir quand il se lance.
Cela permet de lancer plusieur choses en parallèles, comme pour les tests par exemple.
Dans notre cas, les parties ```build``` et ```deploy``` sont les plus importantes.

Mais que fais ce fichier Clara ?

C'est très simple, à chaque événement dans le git, sauf si précisions sur le lancement des jobs (ce que je fais par la suite)
la pipeline se lance sur un ordinateur externe grâce à un [**runner**](https://docs.gitlab.com/runner/).  
Thomas Le Mézo et Marguerite l'ont configuré sur un ordinateur de l'ENSTA au début de projet.
Sur ce runner on a choisi une image docker (ros:humble chez nous) dans laquelle vont s'exécuter les commandes listées dans le fichier yaml.

## Compiler notre projet dans la pipeline
```yaml
build-job:       # This job runs in the build stage, which runs first.
  - echo "Compiling the code..."
  # add other necessary commands for the docker
  - apt update
  - rosdep update --rosdistro humble
  # apt install python3-catkin-pkg
  # build the packages
  - cd ros2_ws/
  - rm -rf build/ install/ log/
  - source /opt/ros/humble/setup.bash
  - colcon build --packages-select robot_pkg
  - colcon build --packages-select vision_pkg
  - colcon build --packages-select benoit-pairob
  - echo "Compile complete."
  rules:
    - if: $CI_COMMIT_BRANCH == "develop"
```

Utilisation de la catégorie ```rules``` : permet de spécifier quel type d'événement déclenche le job.
On peut en mettre plusieur. Ici c'est assez explicite.

## Créer un package debian à partir d'un package ROS2 et l'installer sur son ordinateur

Voir le tutoriel dont sont tirées la plupart des commandes [ici](https://github.com/carlosmccosta/ros_development_tools/blob/master/catkin/create_deb_files_for_ros_packages.md).

Etapes à suivre :

### Bloom

Installer bloom:
```bash
sudo apt-get install python-bloom
sudo apt-get install fakeroot
sudo apt-get install dpkg-dev debhelper
```

* Pour générer des fichiers .deb pour des paquets qui dépendent de paquets qui ne figurent pas dans les listes officielles de rosdep, on doit : 
  * Créer un fichier yaml (dans le workspace contenant les packages, par exemple) spécifiant les packages locaux. Chez nous, c'est ```bloom-config.yml``` qui contient :
    ```
    package_name:
        ubuntu: [benoit-pairob]
    ```
  * Créer un fichier dans les sources de rosdep :
    ```bash
    sudo gedit /etc/ros/rosdep/sources.list.d/10-local.list
    ```
  * Dans lequel on écrit le chemin absolu vers le fichier yaml précédent :
    ```
    yaml file:/home/.../benoit-pai-rob/bloom-config.yaml
    ```
  * On update rosdep:
    ```bash
    rosdep update
    ```
  * Pour vérifier que rosdep trouve le package, on peut utiliser :
    ```bash
    rosdep resolve package_name
    ```


### Generer le fichier .deb

* Source ROS2 et le projet
  ```
  source /opt/ros/foxy/setup.bash
  source ~/.../ros2_ws/install/setup.bash
  ```
* Se déplacer dans le dossier du package (avec le CMakelists.txt et package.xml)
  ```
  cd src/benoit-pairob
  ```
* Update et vérifier que tout va bien avec rosdep :
  ```bash
  rosdep update
  rosdep check --from-paths . --ignore-src --rosdistro="$(rosversion -d)"
  ```
  > Note pour notre package : il faut changer la dépendance ```rviz``` en ```rviz2``` dans le package.xml
  ```bash
  rosdep db | grep package_name
  ```
* Générer un dossier de configuration pour le debian :
  ```bash
  bloom-generate rosdebian --ros-distro $(rosversion -d)
  ```
* Générer le fichier .deb !!! :
  ```bash
  fakeroot debian/rules binary
  ```
* Le fichier .deb est généré dans le dossier parent du package (dossier src)


### Installer le fichier .deb quand on l'a localement
```
sudo dpkg -i ../nomdufichier.deb
```

### Désisntaller le fichier .deb
```
dpkg -l | grep -i benoit
sudo apt remove ros-humble-benoit-pairob
```

> Todo : faire en sorte que les versions s'update automatiquement pour changer le nom du debian notamment !

## La même chose dans le pipeline

```yaml
deploy-job:      # This job runs in the deploy stage.
  stage: deploy  # It only runs when *both* jobs in the test stage complete successfully.
  environment: production
  script:
    - echo "Deploying application..."
    # pour le changement automatique de version
    # catkin_generate_changelog
    # catkin_prepare_release
    # git add src/*/CHANGELOG.rst
    # git commit -m "CHANGELOG.rst"
    - echo "yaml file:/builds/user/benoit-pai-rob/bloom_config.yml" >> /etc/ros/rosdep/sources.list.d/10-local.list # vérfier le path 
    - rosdep update
    - source /opt/ros/humble/setup.bash
    - cd ros2_ws/
    - source install/setup.bash
    - cd src/benoit_pairob/
    - rosdep update
    - echo "Générer dossier debian"
    - bloom-generate rosdebian --ros-distro humble --os-name ubuntu --os-version jammy -d
    - echo "Création du deb"
    - fakeroot debian/rules binary
    - echo "Fichier .deb créé, envoi sur le serveur"
    - cd ..
    - 'curl -u "$ID_CURL:$PSSWRD" -H "Content-Type:multipart/form-data" --data-binary "@$(find . -name *.deb)" "http://172.19.48.50:8081/repository/supernana_dev/"'
    - echo "Application successfully deployed."
  rules:
    - if: $CI_COMMIT_BRANCH == "develop"
```

## Pouvoir installer des packages depuis le serveur Nexus

Ce qu'il faut c'est indiquer à notre ```apt``` l'adresse des sources et les clés de sécurité pour y accéder. Simple non ? Non.
De base, les instructions c'était d'utiliser une commande ```apt-key add ...``` dont le nom est assez explicite mais qui ne fonctionne plus (deprecated).
On utilise donc ```gpg``` de la même manière que pour l'installation de ROS2.

> Note : pour la première méthode, il fallait aussi modifier le fichier /etc/apt/sources.list avec l'URL du serveur.
> 
> Mais maintenant on rajoute un fichier dans le dossier /etc/apt/sources.list.d avec ces informations, cf ci-dessous.

Etapes à suivre :

* Vérifier que l'on a bien le fichier ```dev-public.gpg.key```, sinon me le demander !
* Exécuter la commande suivante :
  ```bash
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/path-to-key/dev-public.gpg.key] http://172.19.48.50:8081/repository/supernana_dev jammy main" | sudo tee /etc/apt/sources.list.d/benoit.list > /dev/null
  ```
  > Note : le nom benoit.list est arbitraire !
* Vérifier si ça marche ! Il faut être connecté à eduroam
  ```bash
  sudo apt update
  sudo apt install ros-humble-benoit-pairob
  ```

## Créer un repository sur le serveur Nexus

Il y a une partie d'utilisation de l'interface de Nexus et une autre de génération de clés de sécurité.

### Création du repository
* Sur le site...

### Génération des clés de sécurité



### Ajout des clés de sécurité au repository

###