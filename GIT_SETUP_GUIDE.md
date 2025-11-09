# Git Setup Guide - Multiple GitHub Accounts

This guide explains how to work with multiple GitHub accounts (personal and company) on the same machine.

## Method 1: SSH Keys (Recommended)

### Step 1: Generate SSH Keys for Each Account

#### For Personal Account (ofu951):
```bash
ssh-keygen -t ed25519 -C "your-personal-email@example.com" -f ~/.ssh/id_ed25519_personal
```

#### For Company Account:
```bash
ssh-keygen -t ed25519 -C "your-company-email@company.com" -f ~/.ssh/id_ed25519_company
```

### Step 2: Add SSH Keys to SSH Agent

```bash
# Start SSH agent
eval "$(ssh-agent -s)"

# Add personal key
ssh-add ~/.ssh/id_ed25519_personal

# Add company key
ssh-add ~/.ssh/id_ed25519_company
```

### Step 3: Add SSH Keys to GitHub

1. Copy your public key:
   ```bash
   # Personal key
   cat ~/.ssh/id_ed25519_personal.pub
   
   # Company key
   cat ~/.ssh/id_ed25519_company.pub
   ```

2. Go to GitHub Settings → SSH and GPG keys → New SSH key
3. Paste the key for each account

### Step 4: Create SSH Config File

Create/edit `~/.ssh/config` (Windows: `C:\Users\YourUsername\.ssh\config`):

```
# Personal GitHub account
Host github.com-personal
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_personal

# Company GitHub account
Host github.com-company
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_company
```

### Step 5: Clone Repository with SSH

For personal account:
```bash
git clone git@github.com-personal:ofu951/UUV_Position_Stabilization.git
```

For company account (when needed):
```bash
git clone git@github.com-company:company-username/repo-name.git
```

### Step 6: Configure Git for Each Repository

#### For Personal Repository:
```bash
cd UUV_Position_Stabilization
git config user.name "Your Personal Name"
git config user.email "your-personal-email@example.com"
```

#### For Company Repository:
```bash
cd company-repo
git config user.name "Your Company Name"
git config user.email "your-company-email@company.com"
```

## Method 2: HTTPS with Personal Access Token

### Step 1: Create Personal Access Token

1. Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate new token with `repo` scope
3. Copy the token (you won't see it again!)

### Step 2: Clone Repository

```bash
git clone https://github.com/ofu951/UUV_Position_Stabilization.git
```

When prompted for credentials:
- Username: `ofu951`
- Password: `your-personal-access-token` (not your GitHub password!)

### Step 3: Configure Git for Repository

```bash
cd UUV_Position_Stabilization
git config user.name "Your Personal Name"
git config user.email "your-personal-email@example.com"
```

## Quick Setup Script

Run this script to quickly set up for personal account:

```bash
# Navigate to where you want to clone
cd C:\Users\YourUsername\Documents  # or your preferred location

# Clone the repository
git clone https://github.com/ofu951/UUV_Position_Stabilization.git

# Navigate into the repository
cd UUV_Position_Stabilization

# Set local git config (only for this repository)
git config user.name "Your Personal Name"
git config user.email "your-personal-email@example.com"

# Verify configuration
git config --list --local
```

## Switching Between Accounts

### Using SSH Method:

1. **For Personal Repos**: Use `github.com-personal` host
2. **For Company Repos**: Use `github.com-company` host

### Using HTTPS Method:

1. **For Personal Repos**: Use personal access token
2. **For Company Repos**: Use company access token

### Check Current Configuration:

```bash
# Check global config
git config --global --list

# Check local config (for current repo)
git config --local --list
```

## Important Notes

1. **Never use global config** for user.name and user.email when working with multiple accounts
2. **Always set local config** for each repository
3. **SSH method is more secure** and doesn't require entering tokens repeatedly
4. **HTTPS method is simpler** but requires token management

## Troubleshooting

### SSH Key Not Working:
```bash
# Test SSH connection
ssh -T git@github.com-personal
ssh -T git@github.com-company
```

### Wrong Account Used:
```bash
# Check current config
git config user.email

# Change to correct account
git config user.email "correct-email@example.com"
```

### Credential Issues:
```bash
# Clear cached credentials (Windows)
git credential-manager-core erase
# or
git credential reject https://github.com
```

## Recommended Workflow

1. Use SSH method for both accounts
2. Set up SSH config file with different hosts
3. Use local git config for each repository
4. Keep global config minimal (only safe.directory if needed)

