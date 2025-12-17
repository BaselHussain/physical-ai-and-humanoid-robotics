#!/bin/bash

echo "========================================="
echo "Testing Neon Connection Pool Fix"
echo "========================================="

# Generate unique email for this test run
TIMESTAMP=$(date +%s)
EMAIL="pool_test_${TIMESTAMP}@test.com"

echo ""
echo "Test 1: Signup new user"
echo "Email: $EMAIL"
SIGNUP_RESPONSE=$(curl -s -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d "{
    \"email\": \"$EMAIL\",
    \"password\": \"TestPass123!\",
    \"background\": {
      \"programming_experience\": \"3-5 years\",
      \"ros2_familiarity\": \"Intermediate\",
      \"hardware_access\": \"Simulation only\"
    }
  }")

echo "$SIGNUP_RESPONSE" | python -m json.tool
TOKEN=$(echo "$SIGNUP_RESPONSE" | python -c "import sys, json; print(json.load(sys.stdin).get('token', ''))" 2>/dev/null)

if [ -z "$TOKEN" ]; then
  echo "❌ FAILED: Signup did not return token"
  exit 1
fi
echo "✅ PASSED: Signup successful"

echo ""
echo "Test 2: Signin with same user"
SIGNIN_RESPONSE=$(curl -s -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d "{
    \"email\": \"$EMAIL\",
    \"password\": \"TestPass123!\"
  }")

echo "$SIGNIN_RESPONSE" | python -m json.tool
if echo "$SIGNIN_RESPONSE" | grep -q "token"; then
  echo "✅ PASSED: Signin successful"
else
  echo "❌ FAILED: Signin did not return token"
  echo "$SIGNIN_RESPONSE"
  exit 1
fi

echo ""
echo "Test 3: Validate session"
SESSION_RESPONSE=$(curl -s -X GET http://localhost:8000/auth/session \
  -H "Authorization: Bearer $TOKEN")

echo "$SESSION_RESPONSE" | python -m json.tool
if echo "$SESSION_RESPONSE" | grep -q "user_id"; then
  echo "✅ PASSED: Session validation successful"
else
  echo "❌ FAILED: Session validation failed"
  exit 1
fi

echo ""
echo "Test 4: Multiple rapid signups (test connection pool)"
for i in {1..5}; do
  echo "  Signup attempt $i..."
  RESP=$(curl -s -X POST http://localhost:8000/auth/signup \
    -H "Content-Type: application/json" \
    -d "{
      \"email\": \"rapid_${TIMESTAMP}_${i}@test.com\",
      \"password\": \"TestPass123!\",
      \"background\": {
        \"programming_experience\": \"0-2 years\",
        \"ros2_familiarity\": \"None\",
        \"hardware_access\": \"None\"
      }
    }")

  if echo "$RESP" | grep -q "token"; then
    echo "  ✅ Signup $i successful"
  else
    echo "  ❌ Signup $i failed"
    echo "$RESP"
    exit 1
  fi
done

echo ""
echo "========================================="
echo "✅ ALL TESTS PASSED!"
echo "Connection pool is working correctly"
echo "========================================="
