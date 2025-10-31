## **生成 SSH 密钥对**

首先，你需要检查你的电脑上是否已经存在 SSH 密钥。

1. **打开终端**
    - 在 Windows 上，推荐使用 `Git Bash`（安装 Git 时会自动安装）。
    - 在 macOS 或 Linux 上，直接使用系统自带的终端 (Terminal)。
2. **检查现有 SSH 密钥**
运行以下命令，查看 `~/.ssh` 目录下是否已有密钥文件：
    
    ```bash
    ls -al ~/.ssh
    ```
    
    如果你看到 `id_rsa` 和 `id_rsa.pub`、`id_ed25519` 和 `id_ed25519.pub` 或类似 `id_*.pub` 的文件，说明你可能已经有密钥了，可以跳到第二步。
    
3. **生成新的 SSH 密钥对**
    
    一般只有在重建容器后才会打开这个文件，请看到这一行时记得设置新容器的git用户名称和邮箱
    ```
    git config --global user.email chenmj75@mail2.sysu.edu.cn
    git config --global user.name minzi9102
    ```
    
    如果没有，或者你想创建一个新的，请运行以下命令。推荐使用 Ed25519 算法，它更安全也更高效。
    
    ```bash
    ssh-keygen -t ed25519 -C chenmj75@mail2.sysu.edu.cn
    ```
    
    将 `"your_email@example.com"` 替换成你注册 GitHub 时使用的邮箱。输入邮箱时不需要带双引号
    
    - 接下来会提示你：
        - `Enter a file in which to save the key (...)`: 直接按 **回车 (Enter)** 键，使用默认路径。
        - `Enter passphrase (empty for no passphrase):`: 输入一个密码。这个密码是用来保护你的私钥文件的，**强烈建议设置一个**。设置后，每次使用该密钥时（例如 `git push`）都需要输入这个密码，而不是你的 GitHub 密码。你也可以直接按回车键跳过，但不推荐。
        - `Enter same passphrase again:`: 再次输入密码确认。
    
    成功后，你会在 `~/.ssh` 目录下看到 `id_ed25519` (私钥) 和 `id_ed25519.pub` (公钥) 两个文件。
    
4. **将 SSH 密钥添加到 ssh-agent**
    
    `ssh-agent` 是一个后台程序，可以帮你管理密钥并记住你的密码。
    
    ```bash
    # 启动 ssh-agent
    eval "$(ssh-agent -s)"
    
    # 将你的私钥添加到 ssh-agent
    # 如果你创建的是 ED25519 密钥
    ssh-add ~/.ssh/id_ed25519
    # 如果你创建的是 RSA 密钥
    # ssh-add ~/.ssh/id_rsa
    ```
    
    如果创建密钥时设置了密码，这里会要求你输入一次，输入后 `ssh-agent` 会在本次会话中记住它。
    

## **将公钥添加到你的 GitHub 账户**

现在需要把你的**公钥** (`.pub` 文件) 的内容告诉 GitHub。

1. **复制公钥内容**
运行以下命令，将公钥文件的内容复制到剪贴板。
    
    ```bash
    cat ~/.ssh/id_ed25519.pub
    ```
    
    然后手动复制全部内容。内容通常以 `ssh-ed25519` 开头，以你的邮箱结尾。
    
2. **登录 GitHub 添加公钥**
    - 登录你的 GitHub 账号。
    - 点击右上角的头像，选择 **Settings**。
    - 在左侧菜单中，点击 **SSH and GPG keys**。
    - 点击 **New SSH key** 或 **Add SSH key** 按钮。
    - **Title**：给这个密钥起一个名字，方便你识别，例如 "My Work Laptop" 或 "VS Code Key"。
    - **Key type**: 保持 `Authentication Key` 即可。
    - **Key**：将刚才复制的公钥内容粘贴到这个文本框里。
    - 点击 **Add SSH key**。GitHub 可能会要求你再次输入登录密码以确认。
3. **测试 SSH 连接**
    
    回到你的终端，运行以下命令来测试连接是否成功：
    
    ```bash
    ssh -T git@github.com
    ```
    
    你可能会看到一个警告，询问你是否信任这个主机，输入 `yes` 并按回车。
    如果连接成功，你会看到这样的消息：
    
    > Hi YourUsername! You've successfully authenticated, but GitHub does not provide shell access.
    > 
    
    看到 `Hi YourUsername` 就说明你已经成功配置好了！
    

## **在 VS Code (或 Git) 中配置使用 SSH**

现在，你的电脑已经和 GitHub 建立起了 SSH 信任关系。最后一步是确保你的项目使用的是 SSH 格式的远程仓库地址。

1. **对于新项目 (Clone)**
    - 在 GitHub 上，找到你想克隆的仓库。
    - 点击绿色的 **`< > Code`** 按钮。
    - 选择 **SSH** 选项卡，而不是 HTTPS。
    - 复制那个以 `git@github.com:` 开头的 URL。
    - 在 VS Code 中，打开命令面板 (`Ctrl+Shift+P` 或 `Cmd+Shift+P`)，输入 `Git: Clone`，然后粘贴这个 SSH URL 进行克隆。
2. **对于已存在的项目 (从 HTTPS 切换到 SSH)**
如果你的项目之前是使用 HTTPS URL 克隆的，你需要更新它的远程仓库地址。
    - 在 VS Code 中，打开你的项目文件夹。
    - 打开终端 (`Ctrl+` ` )。
    - 运行以下命令查看当前的远程地址：
    
    ```bash
    git remote -v
    ```
    
    你会看到类似 `origin https://github.com/YourUsername/YourRepo.git` 的输出。
    
    - 运行以下命令，将其修改为 SSH 地址：
    
    ```bash
    git remote set-url origin git@github.com:YourUsername/YourRepo.git
    ```
    
    修改完成后，VS Code 中的所有 Git 操作（如左侧源代码管理面板的 `同步更改`、`推送`、`拉取`）都会自动通过 SSH 进行，不再需要你输入 GitHub 的用户名和密码了。