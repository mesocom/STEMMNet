#!/bin/bash

# Define the characters that can be used in the password
CHARACTERS="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"

# Initialize the password variable
PASSWORD=""

# Generate a random password by selecting characters from the pool
for i in {1..20}; do
    RANDOM_INDEX=$((RANDOM % ${#CHARACTERS}))
    PASSWORD="${PASSWORD}${CHARACTERS:${RANDOM_INDEX}:1}"
done

# Print the generated password
echo "$PASSWORD"

