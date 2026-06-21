import Stripe from "stripe";

let _stripe: Stripe | null = null;

export function getStripe(): Stripe {
  if (!_stripe) {
    _stripe = new Stripe(process.env.STRIPE_SECRET_KEY!, {
      apiVersion: "2026-05-27.dahlia",
    });
  }
  return _stripe;
}

export const stripe = {
  checkout: { sessions: { create: (...args: Parameters<Stripe["checkout"]["sessions"]["create"]>) => getStripe().checkout.sessions.create(...args) } },
  webhooks: { constructEvent: (...args: Parameters<Stripe["webhooks"]["constructEvent"]>) => getStripe().webhooks.constructEvent(...args) },
};

export const SERVICE_CATEGORIES = [
  "Plumbing",
  "Electrical",
  "Cleaning",
  "Painting",
  "Carpentry",
  "Landscaping",
  "Moving",
  "HVAC",
  "Pest Control",
  "General Handyman",
  "Appliance Repair",
  "Other",
] as const;

export type ServiceCategory = (typeof SERVICE_CATEGORIES)[number];
