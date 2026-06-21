# Deploying FixUsNow to Vercel

## 1. Create a Supabase Database

1. Go to [supabase.com](https://supabase.com) → New project
2. Copy the **Connection string** (Transaction pooler, port 6543) → this is your `DATABASE_URL`

## 2. Set Up Stripe

1. Go to [dashboard.stripe.com](https://dashboard.stripe.com)
2. Copy your **Secret key** → `STRIPE_SECRET_KEY`
3. Copy your **Publishable key** → `STRIPE_PUBLISHABLE_KEY`
4. After deploying, set up a webhook pointing to `https://fixusnow.com/api/stripe/webhook`
   - Events to listen: `checkout.session.completed`
   - Copy the **Webhook signing secret** → `STRIPE_WEBHOOK_SECRET`

## 3. Set Up Google OAuth

1. Go to [console.cloud.google.com](https://console.cloud.google.com)
2. Create OAuth 2.0 credentials
3. Add authorized redirect URI: `https://fixusnow.com/api/auth/callback/google`
4. Copy **Client ID** → `GOOGLE_CLIENT_ID` and **Client Secret** → `GOOGLE_CLIENT_SECRET`

## 4. Deploy to Vercel

1. Push this repo to GitHub
2. Go to [vercel.com](https://vercel.com) → Import Git repository → select `fixusnow`
3. Set **Root Directory** to `fixusnow`
4. Add these Environment Variables:

```
DATABASE_URL=postgresql://...
NEXTAUTH_SECRET=<run: openssl rand -base64 32>
NEXTAUTH_URL=https://fixusnow.com
GOOGLE_CLIENT_ID=...
GOOGLE_CLIENT_SECRET=...
STRIPE_SECRET_KEY=sk_live_...
STRIPE_PUBLISHABLE_KEY=pk_live_...
STRIPE_WEBHOOK_SECRET=whsec_...
```

5. Deploy!

## 5. Run Database Migrations

After first deploy, in Vercel's terminal or locally with DATABASE_URL set:

```bash
npx prisma migrate deploy
```

## 6. Point GoDaddy Domain to Vercel

1. In Vercel: Project → Settings → Domains → Add `fixusnow.com`
2. In GoDaddy DNS: Add/update A record pointing to Vercel's IP (76.76.21.21)
   or add CNAME `www` → `cname.vercel-dns.com`
