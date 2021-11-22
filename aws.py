import time
import boto3

#Values to access AWS
AWS_ACCESS_KEY=""
AWS_SECRET_ACCESS_KEY=""
AWS_REGION="us-east-1"
DYNAMODB_TABLE='deadreckoning'

print("a")
# accesses dynamodb table on aws   
dynamodb = boto3.resource('dynamodb', aws_access_key_id=AWS_ACCESS_KEY,
                          aws_secret_access_key=AWS_SECRET_ACCESS_KEY,
                          region_name =AWS_REGION)

dynamodb_client = boto3.client('dynamodb', aws_access_key_id=AWS_ACCESS_KEY,
                          aws_secret_access_key=AWS_SECRET_ACCESS_KEY,
                          region_name =AWS_REGION)

existing_tables = dynamodb_client.list_tables()['TableNames']

print("1")

if DYNAMODB_TABLE in existing_tables:
    table = dynamodb.Table(DYNAMODB_TABLE)
    table.delete()
    print("EXISTS")

time.sleep(5) #waits for table to delete

print("2")

response = dynamodb_client.create_table(
        AttributeDefinitions=[
            {
                'AttributeName': 'timestamp',
                'AttributeType': 'N',
            }
        ],
        KeySchema=[
            {
                'AttributeName': 'timestamp',
                'KeyType': 'HASH',
            }
        ],
        ProvisionedThroughput={
            'ReadCapacityUnits': 1,
            'WriteCapacityUnits': 1
        },
        TableName=DYNAMODB_TABLE
    )

time.sleep(10) #waits for table to create

#Sends deadreckoning position to dynamodb on AWS
table = dynamodb.Table(DYNAMODB_TABLE)
table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": "52.5",           #sends sonar reading from mbed through serial to dynamodb
        "y": "60.1"          #sends threshold value from mbed to dynamodb
        }
    )
print("test")
